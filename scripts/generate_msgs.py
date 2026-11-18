#!/usr/bin/env python3
"""Generates msg/*.msg from openpilot's cereal schema, so the typed messages mirror openpilot's own.

Field names are openpilot's in snake_case, in openpilot's order, and enums become uint8 constants.
Deprecated fields, unions and raw Data blobs are left out. Re-run after updating openpilot, with a
Python that has pycapnp (the bridge venv from scripts/setup_bridge_venv.sh, or openpilot's own):
  .venv/bin/python3 scripts/generate_msgs.py --openpilot ~/openpilot
"""
import argparse
import glob
import os
import re
import sys
from collections import defaultdict

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MARKER = "# Generated from openpilot"

# Published topic (the openpilot service) -> (ROS message name, Event fields that hold the struct, by schema version)
SERVICES = [
    ("carState", "CarState", ["carState"]),
    ("carControl", "CarControl", ["carControl"]),
    ("carOutput", "CarOutput", ["carOutput"]),
    ("carParams", "CarParams", ["carParams"]),
    ("modelV2", "ModelV2", ["modelV2"]),
    ("longitudinalPlan", "LongitudinalPlan", ["longitudinalPlan"]),
    ("lateralPlan", "LateralPlan", ["lateralPlan", "lateralPlanDEPRECATED"]),
    ("driverStateV2", "DriverStateV2", ["driverStateV2"]),
    ("driverMonitoringState", "DriverMonitoringState", ["driverMonitoringState"]),
    ("extrinsicsCalibration", "ExtrinsicsCalibration", ["extrinsicsCalibration", "liveCalibration"]),
]
PRIMITIVES = {
    "bool": "bool", "int8": "int8", "int16": "int16", "int32": "int32", "int64": "int64",
    "uint8": "uint8", "uint16": "uint16", "uint32": "uint32", "uint64": "uint64",
    "float32": "float32", "float64": "float64", "text": "string",
}
# Every top-level message starts with these, so no openpilot field may use the names
RESERVED = {"header", "log_mono_time", "valid"}


def snake(name):
    s = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)
    return re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", s).lower()


def short(display_name):
    return display_name.split(":")[-1].split(".")[-1]


def skip_reason(name, field):
    if name.lower().endswith("deprecated"):
        return "deprecated"
    if field.proto.discriminantValue != 65535:
        return "union member"
    return None


class Generator:
    def __init__(self, event_schema):
        self.event = event_schema
        self.structs = {}  # struct id -> schema
        self.names = {}  # struct id -> ROS message name
        self.skipped = []
        self.needs_float_array = False

    def struct_fields(self, schema):
        return sorted(schema.fields.items(), key=lambda kv: kv[1].proto.codeOrder)

    def collect(self, schema):
        sid = schema.node.id
        if sid in self.structs:
            return
        self.structs[sid] = schema
        for name, f in schema.fields.items():
            if skip_reason(name, f):
                continue
            if f.proto.which() == "group":
                self.collect(f.schema)
                continue
            t = f.proto.slot.type
            if t.which() == "struct":
                self.collect(f.schema)
            elif t.which() == "list" and t.list.elementType.which() == "struct":
                self.collect(f.schema.elementType)

    def has_fields(self, schema):
        # A group made only of union members or deprecated fields has nothing to mirror
        for name, f in schema.fields.items():
            if skip_reason(name, f):
                continue
            if f.proto.which() == "group" and not self.has_fields(f.schema):
                continue
            return True
        return False

    def assign_names(self):
        tops = {}
        for topic, ros_name, fields in SERVICES:
            field = next((f for f in fields if f in self.event.fields), None)
            if field is None:
                print(f"Skipping {topic}: this openpilot's schema has no {' or '.join(fields)}")
                continue
            s = self.event.fields[field].schema
            tops[s.node.id] = ros_name
            self.collect(s)
        by_short = defaultdict(set)
        for sid, schema in self.structs.items():
            if sid not in tops:
                by_short[short(schema.node.displayName)].add(sid)
        self.names = dict(tops)
        for sid, schema in self.structs.items():
            if sid in self.names:
                continue
            dn = schema.node.displayName
            base = short(dn)
            if len(by_short[base]) > 1 or base in tops.values():
                # Same short name for different structs: prefix with the parent struct
                parent = dn.split(":")[-1].rsplit(".", 1)[0].split(".")[-1]
                base = parent + base[0].upper() + base[1:]
            self.names[sid] = base[0].upper() + base[1:]
        assert len(set(self.names.values())) == len(self.names), "message name clash"
        self.tops = tops

    def enum_constants(self, field_name, enum_schema, width, constants):
        for i, e in enumerate(enum_schema.node.enum.enumerants):
            constants.append(f"{width} {snake(field_name).upper()}_{snake(e.name).upper()}={i}")

    def ros_type(self, name, f, constants):
        t = f.proto.slot.type
        kind = t.which()
        if kind in PRIMITIVES:
            return PRIMITIVES[kind]
        if kind == "enum":
            width = "uint8" if len(f.schema.node.enum.enumerants) <= 256 else "uint16"
            self.enum_constants(name, f.schema, width, constants)
            return width
        if kind == "struct":
            return self.names[f.schema.node.id] if self.has_fields(f.schema) else None
        if kind == "list":
            elem = t.list.elementType
            if elem.which() in PRIMITIVES:
                return PRIMITIVES[elem.which()] + "[]"
            if elem.which() == "enum":
                width = "uint8" if len(f.schema.elementType.node.enum.enumerants) <= 256 else "uint16"
                self.enum_constants(name, f.schema.elementType, width, constants)
                return width + "[]"
            if elem.which() == "struct" and self.has_fields(f.schema.elementType):
                return self.names[f.schema.elementType.node.id] + "[]"
            if elem.which() == "list" and elem.list.elementType.which() == "float32":
                self.needs_float_array = True
                return "FloatArray[]"
        return None

    def render(self, sid):
        schema = self.structs[sid]
        dn = schema.node.displayName
        lines, constants = [], []
        for name, f in self.struct_fields(schema):
            why = skip_reason(name, f)
            if why:
                self.skipped.append(f"{dn}.{name}: {why}")
                continue
            if f.proto.which() == "group":
                if self.has_fields(f.schema):
                    lines.append(f"{self.names[f.schema.node.id]} {snake(name)}")
                else:
                    self.skipped.append(f"{dn}.{name}: only unions or deprecated fields")
                continue
            rt = self.ros_type(name, f, constants)
            if rt is None:
                self.skipped.append(f"{dn}.{name}: {f.proto.slot.type.which()} not mirrored")
                continue
            if sid in self.tops and snake(name) in RESERVED:
                raise SystemExit(f"{dn}.{name} clashes with a field every top-level message has")
            lines.append(f"{rt} {snake(name)}")

        out = [f"{MARKER} {dn} by scripts/generate_msgs.py; do not edit by hand."]
        if sid in self.tops:
            topic = next(t for t, n, _ in SERVICES if n == self.names[sid])
            out += [f"# Published on /<namespace>/{topic}.", "",
                    "std_msgs/Header header",
                    "uint64 log_mono_time  # openpilot Event.logMonoTime, ns",
                    "bool valid  # openpilot Event.valid"]
        if constants:
            out += [""] + constants
        out += [""] + lines
        return "\n".join(out) + "\n"

    def write(self, out_dir):
        os.makedirs(out_dir, exist_ok=True)
        # Replace earlier generated files; hand-written ones like CanMessage.msg stay
        for path in glob.glob(os.path.join(out_dir, "*.msg")):
            with open(path) as fh:
                if fh.readline().startswith(MARKER):
                    os.remove(path)
        written = []
        for sid in sorted(self.structs, key=lambda i: self.names[i]):
            if sid not in self.tops and not self.has_fields(self.structs[sid]):
                continue
            with open(os.path.join(out_dir, f"{self.names[sid]}.msg"), "w") as fh:
                fh.write(self.render(sid))
            written.append(self.names[sid])
        if self.needs_float_array:
            with open(os.path.join(out_dir, "FloatArray.msg"), "w") as fh:
                fh.write(f"{MARKER} lists of float lists by scripts/generate_msgs.py; do not edit by hand.\n"
                         "# One inner list, for example a row of LateralPlan.SolverState.x.\n\nfloat32[] data\n")
            written.append("FloatArray")
        return written


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--openpilot", default=os.environ.get("OPENPILOT_ROOT", os.path.expanduser("~/openpilot")),
                        help="openpilot checkout whose schema to mirror")
    parser.add_argument("--out", default=os.path.join(REPO_ROOT, "msg"), help="output folder for the .msg files")
    args = parser.parse_args()

    op = os.path.abspath(args.openpilot)
    sys.path[:0] = [op, os.path.join(op, "openpilot"), os.path.join(op, "opendbc_repo"), os.path.join(op, "msgq_repo")]
    try:
        from openpilot.cereal import log
    except ImportError:
        # openpilot 0.11.1 and older keep cereal/ at the top of the checkout
        from cereal import log

    gen = Generator(log.Event.schema)
    gen.assign_names()
    written = gen.write(args.out)
    print(f"Wrote {len(written)} messages to {args.out}: {', '.join(written)}")
    print(f"Left out {len(gen.skipped)} fields:")
    for line in gen.skipped:
        print(f"  {line}")


if __name__ == "__main__":
    main()
