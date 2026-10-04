"""Scratch: error de rumbo (yaw del ACK vs real) por tc, en frames rectos (servo ~90)."""
import csv, re, sys, statistics as st
d = sys.argv[1]
for s in sys.argv[2:]:
    sd = f'{d}/s{s}'
    fld = open(f'{sd}/field.txt', encoding='utf-8').readline()
    h0 = float(re.search(r'start=\(-?[\d.]+,-?[\d.]+,(-?[\d.]+)\)', fld).group(1))
    by = {}
    for r in csv.DictReader(open(f'{sd}/trace.csv', encoding='utf-8')):
        if r['pi_frame'] != 'True' or abs(float(r['servo']) - 90) > 6: continue
        m = re.search(r'tc=(\d+).*yaw=(-?[\d.]+)', r['ack'])
        if not m: continue
        e = ((h0 - float(m.group(2))) - float(r['heading']) + 180) % 360 - 180
        by.setdefault(int(m.group(1)), []).append(e)
    print(s, ' '.join(f"{k}:{st.median(v):+.0f}" for k, v in sorted(by.items())))
