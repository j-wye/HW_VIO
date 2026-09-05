#!/usr/bin/env python3
"""Prove a bag's two topic-name stores agree, and that the names are the expected set.

A rosbag2 sqlite3 store names its topics in TWO independent places:
  * the db3 `topics` table  -- what rosbag2_cpp::Reader tags each message with
                               (run_offline/run_feeder dispatch on this)
  * metadata.yaml          -- what `ros2 bag info` prints and what `ros2 bag play`
                               builds its publishers from
Measured 2026-09-03: if they disagree, NOTHING complains. `ros2 bag info` prints the
metadata names, `ros2 bag play` advertises the metadata names and then publishes zero
messages (the storage layer hands it the db3 names, which have no publisher), and
run_offline reports "images=0 imu=0 poses=0" and exits 0. So the skew is invisible
unless something checks for it. This is that check.

--stream additionally prints a sha256 per topic over (timestamp, payload) in playback
order. A rename touches only `topics.name`, so every one of those hashes must be
byte-identical before and after; that is the exact regression test for the rename
itself, and unlike a hash of a VIO trajectory it is genuinely deterministic
(measured: run_offline output is NOT bit-reproducible run-to-run, ~1e-7 relative).
Cost measured: 2.4 s for a 3.65 GB store.

    python3 scripts/verify_bag_names.py datasets/*/            # rc=0 means consistent
    python3 scripts/verify_bag_names.py --expect /camera/image_raw,/imu/data datasets/*/
    python3 scripts/verify_bag_names.py --stream datasets/*/ > /tmp/names_before.txt
"""
import argparse, glob, hashlib, os, re, sqlite3, sys


def stream_hashes(db3):
    """sha256 per topic over (timestamp, payload) in playback order. Invariant under a
    pure topic rename; changes if any message is added, dropped, reordered or edited."""
    con = sqlite3.connect('file:%s?mode=ro' % db3, uri=True)
    names = dict(con.execute('SELECT id,name FROM topics'))
    h = {i: hashlib.sha256() for i in names}
    cur = con.execute('SELECT topic_id,timestamp,data FROM messages ORDER BY timestamp,id')
    while True:
        rows = cur.fetchmany(500)
        if not rows:
            break
        for tid, ts, data in rows:
            d = h[tid]
            d.update(ts.to_bytes(8, 'big'))
            d.update(data)
    con.close()
    return {names[i]: h[i].hexdigest() for i in names}


def check(bag, expect, stream=False):
    bag = bag.rstrip('/')
    db3s = glob.glob(os.path.join(bag, '*.db3'))
    if len(db3s) != 1:
        print(f"FAIL {bag}: expected one .db3, found {db3s}")
        return 1
    con = sqlite3.connect('file:%s?mode=ro' % db3s[0], uri=True)
    db = {n: c for n, c in con.execute(
        'SELECT t.name, COUNT(m.id) FROM topics t LEFT JOIN messages m '
        'ON m.topic_id = t.id GROUP BY t.id')}
    con.close()
    txt = open(os.path.join(bag, 'metadata.yaml')).read()
    # order is not significant: rosbag2_py's writer and the converter's metadata writer
    # emit the list in different orders and both are valid. Compare as a mapping.
    md = dict(zip(re.findall(r'^\s+name: (\S+)$', txt, re.M),
                  (int(x) for x in re.findall(r'^\s+message_count: (\d+)$', txt, re.M)[1:])))
    name = os.path.basename(bag)
    if db != md:
        print(f"SKEW {name}: db3 and metadata.yaml disagree")
        print(f"     only in db3      : {sorted(set(db) - set(md))}")
        print(f"     only in metadata : {sorted(set(md) - set(db))}")
        print(f"     count mismatch   : {sorted(k for k in set(db) & set(md) if db[k] != md[k])}")
        return 1
    missing = [t for t in expect if t not in db]
    if missing:
        print(f"FAIL {name}: expected topic(s) absent: {missing}")
        return 1
    print(f"ok   {name}: {len(db)} topics consistent" +
          (f", expected present: {expect}" if expect else ""))
    if stream:
        for t, hx in sorted(stream_hashes(db3s[0]).items()):
            print(f"     stream {hx[:32]}  {t}  ({db[t]} msgs)")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--expect', default='', help='comma-separated topics that must exist')
    ap.add_argument('--stream', action='store_true',
                    help='also print a per-topic sha256 of (timestamp, payload); '
                         'these must be identical before and after a rename')
    ap.add_argument('bags', nargs='+')
    a = ap.parse_args()
    expect = [t for t in a.expect.split(',') if t]
    sys.exit(sum(check(b, expect, a.stream) for b in a.bags))


if __name__ == '__main__':
    main()
