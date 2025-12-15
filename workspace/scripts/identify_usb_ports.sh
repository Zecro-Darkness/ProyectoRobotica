#!/bin/bash
# Script para identificar puertos USB conectados

echo "========================================="
echo "Identificador de Puertos USB"
echo "========================================="
echo ""

# Listar todos los dispositivos USB
echo "Dispositivos USB conectados:"
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

echo ""
echo "========================================="
echo "Información detallada de cada puerto:"
echo "========================================="

for port in /dev/ttyUSB* /dev/ttyACM*; do
    if [ -e "$port" ]; then
        echo ""
        echo "Puerto: $port"
        udevadm info --name=$port 2>/dev/null | grep -E "ID_VENDOR=|ID_MODEL=|ID_SERIAL=" || echo "  (información no disponible)"
    fi
done

echo ""
echo "========================================="
echo "Sugerencias:"
echo "========================================="
echo "1. Desconecta el Arduino"
echo "2. Ejecuta: ls /dev/ttyUSB*"
echo "3. Conecta el Arduino"
echo "4. Ejecuta de nuevo: ls /dev/ttyUSB*"
echo "5. El nuevo puerto que aparece es el Arduino"
echo ""
echo "Para usar el Arduino con teleop_joint_node:"
echo "  ros2 run phantomx_pincher_classification teleop_joint_node --ros-args -p arduino_port:=/dev/ttyUSBX"
echo ""
