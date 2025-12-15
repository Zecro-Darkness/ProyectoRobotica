# Configuración de Puertos USB

Este documento describe la asignación de puertos USB para el sistema PhantomX Pincher.

## Dispositivos Conectados

### Robot PhantomX Pincher
- **Puerto:** `/dev/ttyUSB1`
- **Chip:** FTDI FT232R USB UART
- **ID_VENDOR:** FTDI
- **ID_MODEL:** FT232R_USB_UART
- **ID_SERIAL:** FTDI_FT232R_USB_UART_AH01FPHW

### Arduino (Control de Relé)
- **Puerto:** `/dev/ttyACM0`
- **Modelo:** Arduino Uno/Mega
- **ID_VENDOR:** Arduino__www.arduino.cc_
- **ID_MODEL:** 0043
- **ID_SERIAL:** Arduino__www.arduino.cc__0043_75932313638351219242

## Configuración Automática

El sistema está configurado para usar automáticamente:
- **Robot:** `/dev/ttyUSB1` (configurado en pincher_control)
- **Arduino:** `/dev/ttyACM0` (configurado en teleop_joint_node)

## Uso

### Modo Normal (Robot + Arduino)
```bash
# Terminal 1 - Lanzar robot
ros2 launch phantomx_pincher_bringup phantomx_pincher.launch.py use_real_robot:=True

# Terminal 2 - Lanzar teleoperación (conecta automáticamente a Arduino en /dev/ttyACM0)
ros2 run phantomx_pincher_classification teleop_joint_node
```

### Deshabilitar Arduino
```bash
ros2 run phantomx_pincher_classification teleop_joint_node \
  --ros-args -p arduino_port:=''
```

### Usar Puerto Diferente
```bash
ros2 run phantomx_pincher_classification teleop_joint_node \
  --ros-args -p arduino_port:=/dev/ttyUSB0
```

## Verificación de Puertos

Para verificar qué dispositivos están conectados:
```bash
/home/juan/Proyecto/KIT_Phantom_X_Pincher_ROS2/scripts/identify_usb_ports.sh
```

## Notas Importantes

- Los puertos se asignan según el puerto físico USB utilizado
- Si siempre conectas los dispositivos a los mismos puertos USB físicos, mantendrán los mismos nombres
- El robot usa un chip FTDI (ttyUSB)
- El Arduino usa CDC-ACM (ttyACM)
