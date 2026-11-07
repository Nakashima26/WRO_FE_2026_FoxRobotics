"""
Digital twin (software-in-the-loop) del Fox para la ronda de obstáculos.

Solo corre en la compu. El runtime de la Pi nunca importa este paquete.

Convenciones compartidas por todos los módulos de twin/:

  Campo (wro_field): origen en el centro del tapete, +x este, +y norte, mm.

  heading del twin (heading_deg): 0 = +y (norte), 90 = +x (este). Crece en
  sentido HORARIO, como una brújula. Es la pose del EJE TRASERO.

  Yaw del ESP32 (anguloGyro, ACK ang= / yaw=): crece en sentido ANTIHORARIO
  (giro a la izquierda = positivo). Una vuelta a la derecha baja el yaw del
  ESP y sube el heading del twin.

  Cuerpo del robot: (derecha_mm, adelante_mm) desde el centro del eje trasero.

  BEV (config.py): 400x400 px, 2 mm/px, x a la derecha, y hacia abajo,
  adelante = -y. (ROBOT_BEV_X, ROBOT_BEV_Y) = (200, 380) es el punto desde el
  que se midieron los marcadores de CALIB_REAL_MM, o sea el EJE DELANTERO
  (config.BEV_ORIGIN_AHEAD_OF_REAR_AXLE_MM).

  Servo: ángulo de firmware 0..180, centro 90, mayor que 90 = izquierda. El
  firmware lo limita a 30..160.

  Motor (TB6612): PWMA duty 0..255. A1=1, A2=0 adelante; A1=0, A2=1 reversa;
  A1=A2=0 coast.
"""
