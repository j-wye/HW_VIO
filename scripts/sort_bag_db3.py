#!/usr/bin/env python3
"""Rewrite a rosbag2 sqlite3 store so rows are inserted in timestamp order.

rosbag2's player assumes insertion order == time order (true for recorded bags).
Our converter stamps rows by header.stamp, which reorders messages relative to
the ROS1 record order, and an out-of-order store makes `ros2 bag play` stall
(images measured at 2-7.5 Hz instead of 10 Hz). This copies the store with
messages re-inserted ORDER BY timestamp -- byte copy only, no decoding.

    python3 scripts/sort_bag_db3.py --bag datasets/AMtown03
"""
import argparse, glob, os, sqlite3, sys


def sort_bag(bagdir):
    db3s = glob.glob(os.path.join(bagdir, '*.db3'))
    if len(db3s) != 1:
        sys.exit(f"expected exactly one .db3 in {bagdir}, found {db3s}")
    src = db3s[0]
    tmp = src + '.sorted'
    if os.path.exists(tmp):
        os.remove(tmp)

    scon = sqlite3.connect(src)
    # already sorted?
    bad, = scon.execute(
        "SELECT COUNT(*) FROM messages a JOIN messages b ON b.id = a.id + 1 "
        "WHERE b.timestamp < a.timestamp").fetchone()
    if bad == 0:
        print(f"[sort] {src}: already in timestamp order, nothing to do")
        return

    dcon = sqlite3.connect(tmp)
    dcon.execute('PRAGMA journal_mode=OFF')
    dcon.execute('PRAGMA synchronous=OFF')
    for (sql,) in scon.execute(
            "SELECT sql FROM sqlite_master WHERE sql IS NOT NULL"):
        dcon.execute(sql)
    # copy every table's rows verbatim except messages (re-inserted sorted below).
    # Humble's plugin refuses to open a store whose `schema` table is empty.
    tables = [r[0] for r in scon.execute(
        "SELECT name FROM sqlite_master WHERE type='table'")]
    for t in tables:
        if t == 'messages':
            continue
        rows = scon.execute(f'SELECT * FROM "{t}"').fetchall()
        if rows:
            ph = ','.join('?' * len(rows[0]))
            dcon.executemany(f'INSERT INTO "{t}" VALUES ({ph})', rows)
    cur = scon.execute(
        'SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp, id')
    n = 0
    while True:
        batch = cur.fetchmany(2000)
        if not batch:
            break
        dcon.executemany(
            'INSERT INTO messages (topic_id, timestamp, data) VALUES (?,?,?)',
            batch)
        n += len(batch)
    dcon.commit()
    dcon.close()
    scon.close()
    os.replace(tmp, src)
    print(f"[sort] {src}: {n} rows re-inserted in timestamp order "
          f"({bad} inversions removed)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True, help='rosbag2 directory')
    sort_bag(ap.parse_args().bag)


if __name__ == '__main__':
    main()
