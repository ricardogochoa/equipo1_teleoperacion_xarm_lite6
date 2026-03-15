# Teleoperación con Sensado de Fuerza Física entre dos xArm Lite 6: Diseño, Implementación y Evaluación Experimental

**Rhett Nieto Ramírez, Valentina González Benedossi, Ricardo Gaspar Ochoa, Óscar Carranza Hernández, Jesús María Valderrama Pérez**  
**Computational Robotics Lab, Tecnológico de Monterrey, Monterrey, Nuevo León, México**  
**{ Emails: A01286100, A00839507, A00838841, A00838649, A01831016}@tec.mx**

---

## Descripción general

Este proyecto presenta el diseño conceptual e implementación de un sistema de **teleoperación maestro–esclavo** entre dos manipuladores **xArm Lite 6**, utilizando **ROS2**, **Python**, **xArm Python SDK**, **UDP** y **ESP32** como parte de la infraestructura de comunicación.

La meta del sistema es permitir que un operador humano mueva un **robot maestro** de forma manual para que un **robot esclavo** replique ese movimiento remotamente en tiempo real. Como extensión del sistema, se plantea integrar **retroalimentación de fuerza física**, de modo que el operador pueda percibir en el manipulador maestro las fuerzas generadas por el esclavo al interactuar con el entorno.

El proyecto combina una base teórica de **control bilateral**, **cinemática diferencial** y **estimación/transmisión de fuerza**, con una implementación práctica orientada a validar la viabilidad del sistema mediante hardware accesible y herramientas modernas de robótica.

---

## Resumen del proyecto

El problema abordado consiste en permitir que un operador controle remotamente un robot esclavo mientras percibe, a través del robot maestro, las fuerzas de contacto generadas durante la interacción con el entorno. La arquitectura propuesta sigue un esquema **maestro–esclavo**, donde la referencia de movimiento del maestro se mapea al esclavo y la fuerza medida o estimada en el efector final del esclavo se transforma en retroalimentación háptica para el operador.

La estrategia de sensado de fuerza considerada en el material base se apoya en la relación entre **fuerza cartesiana** y **torque articular** dada por el **Jacobiano transpuesto**, así como en la posibilidad de implementar un sensor de fuerza de bajo costo con **resortes, potenciómetros y ESP32**.

Experimentalmente, el reto planteado incluye tareas de **contacto** e **inserción tipo peg-in-hole teleoperada**. Aunque en esta etapa aún faltan resultados numéricos específicos de error de posición, fuerza y latencia, la base teórica y arquitectónica muestra que el sistema es viable con hardware accesible y herramientas modernas de ROS2.

---

## Objetivos

### Objetivo general

Diseñar e implementar un sistema de teleoperación maestro–esclavo entre dos robots xArm Lite 6, con base para integrar sensado y retroalimentación de fuerza física.

### Objetivos específicos

- Capturar el movimiento del manipulador maestro en tiempo real.
- Transmitir la referencia articular mediante ROS2 y UDP.
- Replicar el movimiento del maestro en el manipulador esclavo.
- Establecer una infraestructura compatible con retroalimentación de fuerza.
- Validar la arquitectura en tareas de interacción física con el entorno.

---

## Arquitectura general del sistema

```text
xArm Lite 6 Maestro
        ↓
Laptop A (ROS2 + master_node)
        ↓
udp_bridge_A
        ↓
ESP32 Maestro
        ⇄  Enlace inalámbrico / UDP
ESP32 Esclavo
        ↓
udp_bridge_B
        ↓
Laptop B (ROS2 + slave_node)
        ↓
xArm Lite 6 Esclavo
```

### Flujo de operación

1. El operador mueve manualmente el **xArm maestro**.
2. `master_node.py` lee las articulaciones del robot maestro.
3. `udp_bridge_A.py` empaqueta y envía la pose al lado remoto por UDP.
4. El sistema de comunicación basado en ESP32 transmite la información al lado esclavo.
5. `udp_bridge_B.py` recibe la referencia y la publica en ROS2.
6. `slave_node.py` comanda al **xArm esclavo** para replicar el movimiento.
7. Como extensión, la fuerza medida o estimada en el esclavo puede transformarse en realimentación háptica para el maestro.

---

## Tecnologías utilizadas

- **xArm Lite 6**
- **ROS2**
- **Python 3**
- **xArm Python SDK**
- **ESP32**
- **UDP sockets**
- **QoS de ROS2** para teleoperación en tiempo real

---

## Estructura principal del repositorio

```text
.
├── master_node.py
├── slave_node.py
├── udp_bridge_A.py
├── udp_bridge_B.py
├── launch_A.py
├── launch_B.py
├── setup.py
└── README.md
```

> Dependiendo de cómo se organice el paquete ROS2 en el repositorio final, estos archivos pueden encontrarse dentro del paquete `xarm_teleop/` y su carpeta `launch/`.

---

## Descripción de los archivos principales

### `master_node.py`
Nodo ROS2 del lado maestro.

Funciones principales:
- Conectarse al **xArm maestro**.
- Colocar el robot en modo manual (*free-drive / teaching*).
- Leer la posición articular del maestro.
- Publicar las articulaciones en `/teleop/master_joints`.
- Administrar comandos como `START`, `STOP`, `RESUME` y `RESET`.

Características observadas en el código:
- IP del robot maestro definida como `192.168.1.154`.
- Frecuencia de lectura de `30 Hz`.
- Publicación de estado en `/master/status`.

### `udp_bridge_A.py`
Puente ROS2 ↔ UDP del lado maestro.

Funciones principales:
- Recibir comandos por UDP desde el ESP32.
- Publicarlos en ROS2 en `/esp32/cmd`.
- Reenviar mensajes de parada o evento a `/esp32/stop_info`.
- Enviar las articulaciones del maestro al ESP32 con formato tipo `pose,q1,q2,q3,q4,q5,q6`.

### `udp_bridge_B.py`
Puente UDP ↔ ROS2 del lado esclavo.

Funciones principales:
- Escuchar la información proveniente del sistema remoto.
- Publicar la última pose disponible del maestro en `/teleop/master_joints`.
- Publicar comandos y eventos de parada dentro del ecosistema ROS2.

### `slave_node.py`
Nodo ROS2 del lado esclavo.

Funciones principales:
- Conectarse al **xArm esclavo**.
- Suscribirse a `/teleop/master_joints`.
- Replicar el movimiento del maestro usando comandos articulares.
- Aplicar un filtro exponencial para suavizar la referencia.
- Responder a comandos `STOP`, `START`, `RESUME` y `RESET`.

Características observadas en el código:
- IP del robot esclavo definida como `192.168.1.179`.
- Frecuencia de comando de `30 Hz`.
- Uso de `set_servo_angle_j(...)` para seguimiento articular.
- Publicación de estado en `/slave/status`.

### `launch_A.py`
Archivo de lanzamiento del lado maestro.

Inicia:
- `udp_bridge_A`
- `master_node`

### `launch_B.py`
Archivo de lanzamiento del lado esclavo.

Inicia:
- `udp_bridge_B`
- `slave_node`

### `setup.py`
Archivo de configuración del paquete ROS2 en Python.

Define los ejecutables:
- `master_node`
- `slave_node`
- `udp_bridge_A`
- `udp_bridge_B`

---

## Topics ROS2 utilizados

### Publicación

- `/teleop/master_joints` → `sensor_msgs/msg/JointState`
- `/master/status` → `std_msgs/msg/String`
- `/slave/status` → `std_msgs/msg/String`
- `/esp32/cmd` → `std_msgs/msg/String`
- `/esp32/stop_info` → `std_msgs/msg/String`

### Comandos principales

- `START`
- `STOP`
- `RESUME`
- `RESET`

---

## Lógica de operación actual

### Lado maestro

- Espera un comando `START`.
- Entra en modo manual para que el operador pueda mover el brazo con la mano.
- Lee las articulaciones del robot en tiempo real.
- Publica la referencia de movimiento hacia el sistema remoto.

### Lado esclavo

- Recibe la referencia articular.
- Suaviza la señal de entrada para reducir cambios bruscos.
- Ejecuta el movimiento correspondiente en el xArm esclavo.
- Puede detenerse, reanudarse o volver a HOME según los comandos recibidos.

---

## Requisitos

### Hardware

- 2 manipuladores **xArm Lite 6**
- 2 computadoras con **ROS2**
- 2 **ESP32**
- Red WiFi o hotspot local
- Cableado y alimentación adecuados

### Software

- Ubuntu
- ROS2
- Python 3
- `xarm-python-sdk`
- `rclpy`
- `sensor_msgs`
- `std_msgs`
- `colcon`

---

## Instalación

### 1. Clonar el repositorio

```bash
git clone <URL_DEL_REPOSITORIO>
cd <NOMBRE_DEL_REPOSITORIO>
```

### 2. Colocar el paquete en el workspace de ROS2

```bash
mkdir -p ~/dev_ws/src
cp -r <NOMBRE_DEL_REPOSITORIO> ~/dev_ws/src/
cd ~/dev_ws
```

### 3. Instalar dependencias

```bash
pip install xarm-python-sdk
```

### 4. Compilar

```bash
cd ~/dev_ws
colcon build --symlink-install
```

### 5. Cargar el entorno

```bash
source /opt/ros/<distro>/setup.bash
source ~/dev_ws/install/setup.bash
```

---

## Ejecución

La secuencia operativa que se está utilizando actualmente para echar a andar el sistema no depende únicamente de los `launch` del paquete, sino de una ejecución manual distribuida entre la PC maestra y la PC esclava.

## Configuración de red actual

- **PC esclava:** `192.168.137.241`
- **ESP32 esclavo:** `192.168.137.121`
- **PC maestra:** `192.168.137.29`
- **ESP32 maestro:** `192.168.137.187`
- **xArm esclavo:** `192.168.1.179`

> El xArm esclavo se levanta actualmente con `robot_ip:=192.168.1.179`. Si el xArm maestro usa otra IP, debe configurarse directamente en el script correspondiente.

### Orden de arranque actual

#### 1. En la PC esclava, levantar el brazo real con MoveIt

```bash
source /opt/ros/<distro>/setup.bash
source ~/dev_ws/install/setup.bash
ros2 launch xarm_moveit_config lite6_moveit_realmove.launch.py robot_ip:=192.168.1.179
```

#### 2. En la PC maestra, ejecutar el nodo maestro

```bash
python3 master_node.py
```

#### 3. En la PC maestra, enviar el comando `START`

```bash
python3 - <<'PY'
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.sendto(b"START", ("127.0.0.1", 9001))
print("sent START to 127.0.0.1:9001")
PY
```

#### 4. En la PC esclava, iniciar el puente UDP del lado B

```bash
source /opt/ros/<distro>/setup.bash
source ~/dev_ws/install/setup.bash
ros2 run xarm_teleop udp_bridge_B
```

#### 5. En la PC esclava, ejecutar el nodo esclavo

```bash
source /opt/ros/<distro>/setup.bash
source ~/dev_ws/install/setup.bash
ros2 run xarm_teleop slave_node
```

### Resumen rápido de ejecución

```text
PC esclava:
  1) ros2 launch xarm_moveit_config lite6_moveit_realmove.launch.py robot_ip:=192.168.1.179
  4) ros2 run xarm_teleop udp_bridge_B
  5) ros2 run xarm_teleop slave_node

PC maestra:
  2) python3 master_node.py
  3) enviar START por UDP a 127.0.0.1:9001
```

---

## Parámetros relevantes

### IPs configuradas actualmente para la prueba

- **PC esclava:** `192.168.137.241`
- **ESP32 esclavo:** `192.168.137.121`
- **PC maestra:** `192.168.137.29`
- **ESP32 maestro:** `192.168.137.187`
- **xArm esclavo:** `192.168.1.179`

### Puertos usados

- `listen_port = 9001`
- `esp32_port = 9100`
- comando `START` enviado localmente a `127.0.0.1:9001`

---

## Fundamento teórico de la retroalimentación de fuerza

La retroalimentación de fuerza en un sistema maestro–esclavo puede formularse a partir de la relación entre las fuerzas cartesianas en el efector final y los torques articulares del manipulador:

```math
\tau = J^T(q) F
```

donde:
- `\tau` es el vector de torques articulares,
- `J(q)` es el Jacobiano del manipulador,
- `F` es la fuerza cartesiana aplicada en el efector final.

Este principio permite estimar o transformar fuerzas de contacto medidas en el esclavo para posteriormente reproducirlas como señal háptica en el maestro.

Además, el proyecto contempla la posibilidad de implementar un sensor físico de bajo costo basado en:
- resortes,
- potenciómetros,
- microcontroladores ESP32,
- y procesamiento digital en ROS2.

---

## Estado actual del proyecto

### Implementado actualmente

- Arquitectura maestro–esclavo básica.
- Captura de movimiento articular del maestro.
- Reproducción de movimiento en el esclavo.
- Integración ROS2 + UDP + ESP32.
- Manejo de comandos remotos y estados.
- Preparación del sistema para experimentación real.

### Pendiente / trabajo futuro

- Integración completa de retroalimentación de fuerza física.
- Medición formal de latencia extremo a extremo.
- Obtención de métricas experimentales de error de posición y fuerza.
- Evaluación cuantitativa en tareas de contacto e inserción.
- Diseño e integración final del sensor de fuerza de bajo costo.

---

## Posibles aplicaciones

- Teleoperación remota con interacción física.
- Tareas de ensamblaje e inserción.
- Entrenamiento en manipulación robótica.
- Investigación en control bilateral y háptica.
- Plataformas académicas de bajo costo para robótica avanzada.

---

## Limitaciones actuales

- Algunas direcciones IP están definidas directamente en los nodos.
- La realimentación de fuerza aún no está cerrada experimentalmente en esta versión.
- El sistema depende de la estabilidad de la red y sincronización entre nodos.
- La implementación actual está centrada principalmente en teleoperación articular.

---

## Cómo citar este proyecto

```text
Nieto Ramírez, R., González Benedossi, V., Gaspar Ochoa, R., Carranza Hernández, Ó.,
y Valderrama Pérez, J. M. (2026).
Teleoperación con Sensado de Fuerza Física entre dos xArm Lite 6:
Diseño, Implementación y Evaluación Experimental.
Computational Robotics Lab, Tecnológico de Monterrey.
```

---

## Nota final

Este repositorio documenta una base funcional para la teleoperación entre dos manipuladores xArm Lite 6 y sienta las bases para evolucionar hacia un sistema de **teleoperación bilateral con retroalimentación háptica**. La implementación actual valida la viabilidad del enfoque y proporciona la infraestructura necesaria para futuras etapas de evaluación experimental.
