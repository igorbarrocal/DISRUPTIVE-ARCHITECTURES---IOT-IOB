import serial, time
import numpy as np
import pandas as pd
from collections import deque

PORT = "COM3"   # ajuste p/ Linux: /dev/ttyUSB0
BAUD = 115200
N = 200  # tamanho da janela

def extract_feats(x):
    x = x.astype(np.float32)
    return dict(
        mean=float(x.mean()),
        std=float(x.std(ddof=1)),
        rms=float(np.sqrt((x**2).mean())),
        ptp=float(x.max() - x.min())
    )

def raw_to_unit(raw): return raw / 4095.0

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2.0)

rows = []
buf1, buf2 = deque(maxlen=N), deque(maxlen=N)

print("Coletando… CTRL+C para parar")
try:
    while True:
        line = ser.readline().decode(errors="ignore").strip()
        if not line or line.startswith("#"): continue
        seq, ts_ms, sample, r1, r2 = line.split(',')
        r1, r2 = int(r1), int(r2)

        buf1.append(r1); buf2.append(r2)

        if len(buf1) == N:
            x = np.array(buf1)/4095.0
            z = np.array(buf2)/4095.0
            feats = extract_feats(x)
            feats['seq'], feats['pot2_mean'] = int(seq), float(z.mean())
            rows.append(feats)

            # Simulação de limite
            TH_ON  = max(0.05, min(0.95, feats['pot2_mean']))
            TH_OFF = max(0.0, TH_ON - 0.1)
            score  = feats['rms']

            leak = getattr(extract_feats, "_leak", False)
            if not leak and score >= TH_ON:
                ser.write(b"LED ON\n"); extract_feats._leak = True
            elif leak and score <= TH_OFF:
                ser.write(b"LED OFF\n"); extract_feats._leak = False

            print(f"[seq {seq}] rms={score:.3f} "
                  f"THon={TH_ON:.2f} THoff={TH_OFF:.2f} "
                  f"LED={'ON' if getattr(extract_feats,'_leak',False) else 'OFF'}")

            buf1.clear(); buf2.clear()

except KeyboardInterrupt:
    pass
finally:
    ser.close()

if rows:
    df = pd.DataFrame(rows)
    df.to_csv("features_aula1.csv", index=False)
    print("Arquivo salvo: features_aula1.csv")