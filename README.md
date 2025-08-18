# luva-leorover-controle
Código em arduino que possibilita o controle do robô Leo Rover com a gesticulação dos dedos com a luva Hiwonder.

### Setup
O código deve ser aberto no Arduino IDE e ser feito o upload dele para a luva Hiwonder. Para movimentar o Leo Rover o computador deve estar conectado ao USB da luva e com o rover através do wi-fi.

Abra o terminal e execute o seguinte código:

```
rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600
```

Os gestos para mover o rover dependem se apenas um dos dedos está levantado. Os LEDs indicam se eles estão levantados ou não, acendidos se estão levantados e desligados se estão fechados. 

### Gestos de movimento:

- Frente: Indicador levantado
- Trás: Mão fechada (todos os dedos fechados)
- Esquerda: Polegar levantado
- Direita: Mínimo levantado
- Parado: Mais que um dedo levantado ou todos levantados


