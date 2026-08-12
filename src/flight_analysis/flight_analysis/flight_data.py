#!/usr/bin/env python3
"""Read a flight bag, show the raw numbers, plot vicon-vs-onboard position.

Reads the bag directly, with no `ros2 unbag` / CSV step in between. That
matters for more than convenience: the CSV export takes each row's time from
`header.stamp`, and vicon_receiver/msg/Position has no header field at all, so
exported vicon timestamps are the time the *export* ran, not capture time.
Reading the bag hands back each message together with its own recv timestamp,
so that failure mode cannot occur here.

Run it with no arguments and it picks the most recent bag by itself. Figures
are always written into the bag folder as PNGs rather than shown, so nothing
depends on a working matplotlib GUI backend; pass --show to also open windows.

    ros2 run flight_analysis flight_data
    ros2 run flight_analysis flight_data ~/biodrone/bags/flight_20260723_164231
    ros2 run flight_analysis flight_data --dump
    ros2 run flight_analysis flight_data --dump --topics /cf09/motor_pwm
"""

import argparse
import json
import re
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

BAGS_DIR = Path.home() / "biodrone" / "bags"

# stateEstimate vars in the order declared by the `est` log block in
# crazyflies.yaml. LogDataGeneric carries a bare float array with no field
# names, so this ordering is the only thing that maps values -> meaning.
EST_VARS = ["x", "y", "z", "roll", "pitch", "yaw"]

# Colorblind-safe, fixed assignment so "vicon" and "est" always mean the same
# colour in every figure.
COLOR_VICON = "#0072B2"
COLOR_EST = "#D55E00"


def bag_start_ns(bag_dir: Path) -> int:
    """Start time and message count from the bag's own metadata.

    An empty bag reports starting_time = INT64_MAX (year 2262) rather than
    anything sane, so the count is what tells you the value is meaningful.
    """
    m = rosbag2_py.Info().read_metadata(str(bag_dir), "sqlite3")
    return m.starting_time.nanoseconds, m.message_count


def newest_bag(bags_dir: Path = BAGS_DIR):
    """The most recent *flight*, by the bag's recorded start time.

    Returns (bag_dir, skipped) where `skipped` lists empty bags that were
    passed over, newest first, so the caller can say so out loud.

    Deliberately not folder mtime: this tool writes its PNGs and CSVs into the
    bag folder, so mtime tracks "last analysed" rather than "last flown" and
    would keep re-selecting whichever bag you looked at most recently.

    Bags that recorded nothing are skipped. Killing a launch before the node
    timers fire leaves exactly that, and their INT64_MAX start time would
    otherwise sort them above every real flight. Passing one explicitly still
    works.

    A bag is the folder, not the .db3 inside it -- metadata.yaml is what lists
    the split _0/_1 chunks, so folders without one are skipped.
    """
    if not bags_dir.is_dir():
        sys.exit(f"no bags directory at {bags_dir}")
    dated, empty = [], []
    for p in sorted(bags_dir.iterdir()):
        if not (p.is_dir() and (p / "metadata.yaml").exists()):
            continue
        try:
            start_ns, count = bag_start_ns(p)
        except Exception:
            continue  # unreadable/corrupt metadata -- not a candidate
        (dated if count else empty).append((start_ns, p))
    if not dated:
        sys.exit(f"no usable bags under {bags_dir}"
                 + (f" -- all {len(empty)} bag(s) there recorded 0 messages"
                    if empty else " (no bag folders found at all)"))
    chosen = max(dated)[1]
    # Empty bags carry no usable start time, so order them by name -- which is
    # a recording timestamp under the launch file's flight_%Y%m%d_%H%M%S.
    skipped = [p for _, p in sorted(empty, key=lambda kp: kp[1].name, reverse=True)]
    return chosen, skipped


def open_reader(bag_dir: Path) -> rosbag2_py.SequentialReader:
    """A reader positioned at the start of the bag, plus nothing else."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    return reader


def topic_types(bag_dir: Path) -> dict:
    """Topic name -> message type string, for every topic in the bag."""
    reader = open_reader(bag_dir)
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def find_drone(types: dict) -> str:
    """Infer the drone name from the topics the bag actually contains.

    Looks for a name that has both a `/vicon/<name>/<name>` and a `/<name>/est`
    topic, since those are the two this tool compares. Falls back to any drone
    with an /est topic so the raw dump still works on a bag with no vicon.
    """
    vicon = {m.group(1) for t in types
             if (m := re.fullmatch(r"/vicon/([^/]+)/\1", t))}
    est = {m.group(1) for t in types
           if (m := re.fullmatch(r"/([^/]+)/est", t))}
    both = sorted(vicon & est)
    if both:
        return both[0]
    if est:
        return sorted(est)[0]
    sys.exit("no /<drone>/est topic in this bag -- pass --drone to force a name")


def bag_summary(bag_dir: Path) -> pd.DataFrame:
    """Every topic with its type and message count, like `ros2 bag info`.

    A count of 0 means that topic recorded nothing -- usually a QoS mismatch
    between publisher and recorder -- which is worth seeing before you trust
    any plot built from the bag.
    """
    types = topic_types(bag_dir)
    counts = dict.fromkeys(types, 0)
    reader = open_reader(bag_dir)
    while reader.has_next():
        topic, _, _ = reader.read_next()
        counts[topic] += 1
    return pd.DataFrame([{"topic": n, "type": types[n], "messages": counts[n]}
                         for n in sorted(types)])


def read_topic(bag_dir: Path, topic: str) -> pd.DataFrame:
    """One row per message, message fields as columns, plus `t_ns`.

    `t_ns` is the bag's own recv time in nanoseconds since the epoch, returned
    by the same read that produced the message -- the two cannot drift apart.
    Returns an empty frame if the topic isn't in the bag or recorded nothing.
    """
    types = topic_types(bag_dir)
    if topic not in types:
        return pd.DataFrame()
    msg_cls = get_message(types[topic])
    reader = open_reader(bag_dir)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    rows = []
    while reader.has_next():
        _, data, t_ns = reader.read_next()
        msg = deserialize_message(data, msg_cls)
        row = {"t_ns": t_ns}
        row.update({f: getattr(msg, f) for f in msg.get_fields_and_field_types()})
        rows.append(row)
    return pd.DataFrame(rows)


def topic_filename(bag_dir: Path, topic: str, ext: str) -> str:
    """`/cf09/est` in bag `flight_X` -> `flight_X_cf09_est.<ext>`."""
    return f"{bag_dir.name}_{topic.strip('/').replace('/', '_')}.{ext}"


def read_all(bag_dir: Path, topics=None) -> dict:
    """Every message of every (selected) topic, as plain nested dicts.

    One pass over the bag rather than one per topic, and each record carries
    `t_ns` (the bag's own recv time) as its first key. Message contents come
    from message_to_ordereddict, so nested types survive intact -- which is the
    point of dumping to JSON rather than flattening to a table.
    """
    types = topic_types(bag_dir)
    if topics:
        unknown = sorted(set(topics) - set(types))
        if unknown:
            sys.exit(f"not in this bag: {', '.join(unknown)}")
    wanted = [t for t in types if not topics or t in topics]
    classes = {t: get_message(types[t]) for t in wanted}
    out = {t: [] for t in wanted}

    reader = open_reader(bag_dir)
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        rec = {"t_ns": t_ns}
        rec.update(message_to_ordereddict(deserialize_message(data, classes[topic])))
        # LogDataGeneric's `values` is a bare array whose meaning lives in the
        # log block's `vars`, not in the message. Name the entries so a dump of
        # /<drone>/est is readable without cross-referencing crazyflies.yaml.
        if topic.endswith("/est") and isinstance(rec.get("values"), (list, tuple)):
            rec.update(dict(zip(EST_VARS, rec["values"])))
        out[topic].append(rec)
    return out


def dump(bag_dir: Path, out_dir: Path, fmt: str, topics=None) -> None:
    """Write every selected topic to out_dir, one file per topic."""
    out_dir.mkdir(parents=True, exist_ok=True)
    data = read_all(bag_dir, topics)
    for topic, records in data.items():
        if not records:
            print(f"  skipped {topic} (0 messages)")
            continue
        out = out_dir / topic_filename(bag_dir, topic, fmt)
        if fmt == "json":
            # default=list catches array.array / numpy fields, which json
            # cannot encode on its own.
            out.write_text(json.dumps(records, indent=2, default=list))
        else:
            # Nested fields become dotted columns (header.stamp.sec, ...);
            # arrays stay as one cell, which is why JSON is the better dump.
            pd.json_normalize(records).to_csv(out, index=False)
        print(f"  wrote {out}  ({len(records)} messages)")


def named_est(est: pd.DataFrame) -> pd.DataFrame:
    """Replace LogDataGeneric's anonymous `values` array with named columns."""
    if est.empty or "values" not in est:
        return est
    arr = np.stack(est["values"].to_numpy())
    out = est.drop(columns=["values"]).copy()
    for i, var in enumerate(EST_VARS[:arr.shape[1]]):
        out[var] = arr[:, i]
    return out


def positions(vicon: pd.DataFrame, est: pd.DataFrame):
    """Both position traces in metres, on a shared clock starting at zero.

    Vicon reports translation in MILLIMETRES while stateEstimate is already in
    metres; overlaying them raw leaves one trace a flat line 1000x off the
    other.
    """
    t0 = min(vicon["t_ns"].min(), est["t_ns"].min())
    tv = ((vicon["t_ns"] - t0) / 1e9).to_numpy()
    te = ((est["t_ns"] - t0) / 1e9).to_numpy()
    vicon_xyz = {a: vicon[f"{a}_trans"].to_numpy() / 1000.0 for a in "xyz"}
    est_xyz = {a: est[a].to_numpy() for a in "xyz"}
    return tv, vicon_xyz, te, est_xyz


def plot_overlay(tv, vicon_xyz, te, est_xyz, drone: str):
    """Raw traces, one stacked subplot per axis."""
    fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    for ax, a in zip(axes, "xyz"):
        ax.plot(tv, vicon_xyz[a], color=COLOR_VICON, lw=1.25, label="vicon")
        ax.plot(te, est_xyz[a], color=COLOR_EST, lw=1.25, label="est")
        ax.set_ylabel(f"{a} (m)")
        ax.grid(True, alpha=0.25)
    axes[0].legend(loc="upper right", frameon=False)
    axes[-1].set_xlabel("time (s)")
    fig.suptitle(f"{drone} position: vicon vs onboard estimate")
    fig.tight_layout()
    return fig


def plot_difference(tv, vicon_xyz, te, est_xyz, drone: str):
    """est - vicon per axis, plus the 3D error magnitude.

    The topics run at different rates (est is 10 Hz per the log block, vicon is
    whatever the mocap bridge pushes), so vicon is linearly interpolated onto
    the est timestamps before subtracting. Interpolation is only meaningful
    inside vicon's own time span, so est samples outside it are dropped --
    holding flat at the endpoints instead would invent error that isn't real.
    """
    inside = (te >= tv.min()) & (te <= tv.max())
    if not inside.any():
        sys.exit("vicon and est timestamps do not overlap -- nothing to compare")
    t = te[inside]
    diff = {a: est_xyz[a][inside] - np.interp(t, tv, vicon_xyz[a]) for a in "xyz"}
    norm = np.sqrt(sum(diff[a] ** 2 for a in "xyz"))

    print(f"\n=== position error, est - vicon (n={len(t)}) ===")
    for a in "xyz":
        d = diff[a]
        print(f"  {a}:   mean {d.mean():+.4f}   std {d.std():.4f}   "
              f"max|.| {np.abs(d).max():.4f}   (m)")
    print(f"  |d|: mean {norm.mean():.4f}   max {norm.max():.4f}   (m)")
    if inside.sum() < len(te):
        print(f"  ({len(te) - inside.sum()} est samples outside vicon's time span, dropped)")

    fig, axes = plt.subplots(4, 1, figsize=(9, 9), sharex=True)
    for ax, a in zip(axes, "xyz"):
        ax.axhline(0, color="0.6", lw=0.8)
        ax.plot(t, diff[a], color=COLOR_EST, lw=1.25)
        ax.set_ylabel(f"d{a} (m)")
        ax.grid(True, alpha=0.25)
    axes[3].plot(t, norm, color="0.2", lw=1.25)
    axes[3].set_ylabel("|d| (m)")
    axes[3].grid(True, alpha=0.25)
    axes[3].set_xlabel("time (s)")
    fig.suptitle(f"{drone} position error: est - vicon "
                 "(vicon interpolated onto est times)")
    fig.tight_layout()
    return fig


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", nargs="?",
                    help="bag folder (default: newest non-empty one in --bags-dir)")
    ap.add_argument("--bags-dir", metavar="DIR", default=str(BAGS_DIR),
                    help=f"where to look for the newest bag (default: {BAGS_DIR})")
    ap.add_argument("--drone", help="drone name (default: inferred from the bag)")
    dump_group = ap.add_mutually_exclusive_group()
    dump_group.add_argument("--dump", dest="dump", action="store_true",
                            help="write the full debug dump: every topic, "
                                 "every message, one file per topic")
    dump_group.add_argument("--no-dump", dest="dump", action="store_false",
                            help="skip the dump (the default)")
    ap.set_defaults(dump=False)
    ap.add_argument("--dump-format", choices=("json", "csv", "both"),
                    default="json",
                    help="dump format (default: json). CSV flattens nested "
                         "fields to dotted columns and puts arrays in a single "
                         "cell, so prefer json for anything but plain numeric "
                         "topics")
    ap.add_argument("--topics", nargs="+", metavar="TOPIC",
                    help="restrict the dump to these topics (default: all)")
    ap.add_argument("--out", metavar="DIR",
                    help="where every generated file goes -- figures and dump "
                         "alike (default: the bag folder, alongside the .db3)")
    ap.add_argument("--show", action="store_true",
                    help="also open the figures in a window; off by default "
                         "because the saved PNGs don't depend on a working "
                         "matplotlib GUI backend")
    ap.add_argument("--rows", type=int, default=10,
                    help="raw rows to print per topic (default 10)")
    args = ap.parse_args()

    if args.bag:
        bag_dir = Path(args.bag).expanduser()
        if not (bag_dir / "metadata.yaml").exists():
            sys.exit(f"not a bag folder (no metadata.yaml): {bag_dir}")
        print(f"bag: {bag_dir}")
    else:
        bag_dir, skipped = newest_bag(Path(args.bags_dir).expanduser())
        print(f"bag: {bag_dir}")
        if skipped:
            # Killing a launch early records nothing, which is easy to do by
            # accident -- name them so a run that "worked" isn't mistaken for
            # a flight that was never captured.
            print(f"\nwarning: skipped {len(skipped)} empty bag(s) newer than "
                  "or alongside this one -- they recorded 0 messages:")
            for p in skipped[:5]:
                print(f"  {p.name}")
            if len(skipped) > 5:
                print(f"  ... and {len(skipped) - 5} more")

    print("\n=== topics ===")
    summary = bag_summary(bag_dir)
    print(summary.to_string(index=False))

    if summary.empty or not summary["messages"].any():
        sys.exit("\nwarning: this bag is EMPTY -- 0 messages on every topic. "
                 "That's what a launch killed before the node timers fire "
                 "leaves behind; there is nothing to analyse.")
    silent = summary.loc[summary["messages"] == 0, "topic"].tolist()
    if silent:
        # A subscribed-but-silent topic is nearly always a QoS mismatch
        # (recorder defaults to RELIABLE, publisher is BEST_EFFORT).
        print(f"\nwarning: {len(silent)} topic(s) recorded 0 messages: "
              f"{', '.join(silent)}")

    drone = args.drone or find_drone(topic_types(bag_dir))
    vicon_topic, est_topic = f"/vicon/{drone}/{drone}", f"/{drone}/est"
    print(f"\ndrone: {drone}")

    vicon = read_topic(bag_dir, vicon_topic)
    est = named_est(read_topic(bag_dir, est_topic))

    pd.set_option("display.width", 200)
    pd.set_option("display.max_columns", 50)
    for name, df in ((vicon_topic, vicon), (est_topic, est)):
        print(f"\n=== {name} raw ({len(df)} messages) ===")
        print("EMPTY -- nothing recorded on this topic" if df.empty
              else df.head(args.rows).to_string(index=False))

    # One output location for everything -- dump and figures land together in
    # the bag folder unless --out moves them, so a flight's artefacts never end
    # up split across two places. Names are bag-prefixed, so even a shared
    # --out directory stays unambiguous and re-running overwrites in place.
    out_dir = Path(args.out).expanduser() if args.out else bag_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.dump:
        formats = ("json", "csv") if args.dump_format == "both" else (args.dump_format,)
        for fmt in formats:
            print(f"\n=== {fmt} dump ===")
            dump(bag_dir, out_dir, fmt, args.topics)
    elif args.topics or args.dump_format != "json":
        print("\nnote: --topics/--dump-format do nothing without --dump")

    if vicon.empty or est.empty:
        sys.exit(f"\ncannot compare: need messages on both {vicon_topic} "
                 f"and {est_topic}")

    tv, vicon_xyz, te, est_xyz = positions(vicon, est)
    figs = {
        "position": plot_overlay(tv, vicon_xyz, te, est_xyz, drone),
        "error": plot_difference(tv, vicon_xyz, te, est_xyz, drone),
    }

    print()
    for kind, fig in figs.items():
        out = out_dir / f"{bag_dir.name}_{kind}.png"
        fig.savefig(out, dpi=120)
        print(f"wrote {out}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
