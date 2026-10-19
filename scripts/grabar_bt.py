"""
Graba el log de VueltaCalibracion (o RondaAbierta) por Bluetooth a un archivo.

Antes: empareja la PC con el ESP32 en Configuración de Windows -> Bluetooth.
Windows le crea puertos COM; el que sirve es el que lleva la dirección del
ESP32 (ESP32_MAC). Sin puerto, el script lo busca solo.

Uso:
  python scripts/grabar_bt.py                 busca el ESP32 y graba
  python scripts/grabar_bt.py COM9            por otro puerto (COM9 = cable USB)
  python scripts/grabar_bt.py --lista         lista los puertos
Mientras graba: g + Enter arranca el carro, x + Enter lo para, Ctrl+C termina.

El archivo queda en logs_tof/ (en la raíz del repo) con fecha y hora.
"""
import argparse
import datetime
import os
import sys
import threading

import serial
import serial.tools.list_ports

# Dirección Bluetooth del ESP32 del carro (sale en la app del celular / en Windows).
ESP32_MAC = "1CC3ABC14462"
RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def buscar_puerto():
    """El puerto COM de salida hacia el ESP32: el que lleva su dirección en el hwid.
    Los de entrada traen 000000000000 y no sirven."""
    for p in serial.tools.list_ports.comports():
        if ESP32_MAC in p.hwid.upper():
            return p.device
    return None


def main():
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("puerto", nargs="?", help="puerto COM del ESP32 (ej. COM7)")
    ap.add_argument("-o", "--salida", help="archivo de salida (default logs_tof/vuelta_FECHA.txt)")
    ap.add_argument("--baud", type=int, default=115200, help="solo importa por USB")
    ap.add_argument("--lista", action="store_true", help="lista los puertos COM y sale")
    args = ap.parse_args()

    if args.lista:
        for p in serial.tools.list_ports.comports():
            nota = "  <- ESP32 del carro" if ESP32_MAC in p.hwid.upper() else ""
            print(f"{p.device:6s} {p.description}{nota}")
        return
    if not args.puerto:
        args.puerto = buscar_puerto()
        if not args.puerto:
            sys.exit("No encontré el ESP32 entre los puertos Bluetooth. ¿Está emparejado en Windows?\n"
                     "Mira los puertos con --lista, o pasa uno a mano (COM9 = cable USB).")

    salida = args.salida
    if not salida:
        os.makedirs(os.path.join(RAIZ, "logs_tof"), exist_ok=True)
        salida = os.path.join(RAIZ, "logs_tof", datetime.datetime.now().strftime("vuelta_%Y%m%d_%H%M%S.txt"))

    # Reintenta hasta que el ESP32 aparezca: así se corre esto primero y luego se
    # prende el carro (RondaAbierta arranca sola a los 5 s y no da tiempo de conectar).
    print(f"Conectando a {args.puerto}... (si el carro está apagado, préndelo; Ctrl+C para salir)")
    intentos = 0
    while True:
        try:
            ser = serial.Serial(args.puerto, args.baud, timeout=0.5)
            break
        except serial.SerialException as e:
            intentos += 1
            if intentos == 1 or intentos % 10 == 0:
                print(f"  esperando al ESP32... ({e.__class__.__name__}; si sigue así: ¿otra app,"
                      " como el monitor serie o el celular, ya está conectada?)")
        except KeyboardInterrupt:
            sys.exit("\nCancelado.")
    print(f"Conectado. Grabando en {salida}")
    print("Escribe g + Enter para arrancar (VueltaCalibracion), x + Enter para parar, Ctrl+C para salir.\n")

    # Lo que escribas en la terminal se manda al ESP32.
    def teclado():
        for linea in sys.stdin:
            ser.write(linea.strip().encode())

    threading.Thread(target=teclado, daemon=True).start()

    n_t = n_p = 0
    with open(salida, "w", encoding="utf-8") as f:
        try:
            while True:
                linea = ser.readline().decode("utf-8", errors="replace").rstrip("\r\n")
                if not linea:
                    continue
                f.write(linea + "\n")
                if linea.startswith("T,"):
                    n_t += 1
                elif linea.startswith("P,"):
                    n_p += 1
                    if n_p % 50 == 0:
                        print(f"\r  posiciones: {n_p}  lecturas ToF: {n_t}   ", end="", flush=True)
                else:
                    print("\n" + linea if n_p else linea)
        except KeyboardInterrupt:
            pass
    ser.close()
    print(f"\nListo: {n_p} posiciones, {n_t} lecturas ToF en {salida}")
    print(f"Analízalo: python scripts/analiza_vuelta.py {salida}")


if __name__ == "__main__":
    main()
