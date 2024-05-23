
#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/supervisor.h>

#define TIME_STEP 256
#define QtddSensoresProx 8
#define QtddLeds 10
#define respiro 0.001

int main(int argc, char **argv) {
  int i;
  double LeituraSensorProx[QtddSensoresProx];
  double motorEsquerdoVel = 1.0, motorDireitoVel = 1.0;
 

  wb_robot_init();
  
  
  WbNodeRef caixa = wb_supervisor_node_get_from_def("woodenBox");
  WbNodeRef e_puck = wb_supervisor_node_get_from_def("e-puck");
  WbFieldRef movimento_caixa = wb_supervisor_node_get_field(caixa, "translation");
  WbFieldRef movimento_e_puck = wb_supervisor_node_get_field(e_puck, "translation");
 
  
  double posicaoInicialCaixa[3];
  const double *posicaoCaixa;

  posicaoCaixa = wb_supervisor_field_get_sf_vec3f(movimento_caixa);
  for (int i = 0; i < 3; i++) {
    posicaoInicialCaixa[i] = posicaoCaixa[i];
  }
  WbDeviceTag MotorEsquerdo, MotorDireito;
  MotorEsquerdo = wb_robot_get_device("left wheel motor");
  MotorDireito = wb_robot_get_device("right wheel motor");

  wb_motor_set_position(MotorEsquerdo, INFINITY);
  wb_motor_set_position(MotorDireito, INFINITY);

  wb_motor_set_velocity(MotorEsquerdo, 0);
  wb_motor_set_velocity(MotorDireito, 0);

  WbDeviceTag SensorProx[QtddSensoresProx];
  for (i = 0; i < QtddSensoresProx; i++) {
    char sensor_name[4];
    sprintf(sensor_name, "ps%d", i);
    SensorProx[i] = wb_robot_get_device(sensor_name);
    wb_distance_sensor_enable(SensorProx[i], TIME_STEP);
  }

  WbDeviceTag Leds[QtddLeds];
  Leds[0] = wb_robot_get_device("led0");
  Leds[1] = wb_robot_get_device("led1");
  Leds[2] = wb_robot_get_device("led2");
  Leds[3] = wb_robot_get_device("led3");
  Leds[4] = wb_robot_get_device("led4");
  Leds[5] = wb_robot_get_device("led5");
  Leds[6] = wb_robot_get_device("led6");
  Leds[7] = wb_robot_get_device("led7");
  Leds[8] = wb_robot_get_device("led8");
  Leds[9] = wb_robot_get_device("led9");
  wb_led_set(Leds[0], -1);

  int contadorVezesPreso = 0;
  int contadorParaForcarRobo = 0;
  const double *posicaoRobo;
  double posicaoAnterior[3] = {0, 0, 0};
  int steps_stuck = 10; 

  while (wb_robot_step(TIME_STEP) != -1) {
    
    posicaoRobo = wb_supervisor_field_get_sf_vec3f(movimento_e_puck);
  
    for (int i = 0; i < 3; i++) {
    posicaoAnterior[i] = posicaoRobo[i];
  }
  
    posicaoCaixa = wb_supervisor_field_get_sf_vec3f(movimento_caixa);
     if (posicaoCaixa[0] > posicaoInicialCaixa[0] + respiro ||
        posicaoCaixa[0] < posicaoInicialCaixa[0] - respiro ||
        posicaoCaixa[1] > posicaoInicialCaixa[1] + respiro ||
        posicaoCaixa[1] < posicaoInicialCaixa[1] - respiro ||
        posicaoCaixa[2] > posicaoInicialCaixa[2] + respiro ||
        posicaoCaixa[2] < posicaoInicialCaixa[2] - respiro) {
        wb_led_set(Leds[1], 1);
        wb_led_set(Leds[2], 1);
        wb_led_set(Leds[3], 1);
        wb_led_set(Leds[4], 1);
        wb_led_set(Leds[5], 1);
        wb_led_set(Leds[6], 1);
        wb_led_set(Leds[7], 1);
        wb_led_set(Leds[8], 1);
        wb_led_set(Leds[9], 1);
        wb_motor_set_velocity(MotorEsquerdo, 0);
        wb_motor_set_velocity(MotorDireito, 0);
        break;
      }

    for (i = 0; i < QtddSensoresProx; i++) {
      LeituraSensorProx[i] = wb_distance_sensor_get_value(SensorProx[i]);
    }

    wb_led_set(Leds[0], wb_led_get(Leds[0]) * -1);

    double distancia = 100; 


    bool obstaculo_frente = LeituraSensorProx[0] > distancia || LeituraSensorProx[1] > distancia || LeituraSensorProx[2] > distancia;
    bool obstaculo_direita = LeituraSensorProx[3] > distancia || LeituraSensorProx[4] > distancia;
    bool obstaculo_esquerda = LeituraSensorProx[5] > distancia || LeituraSensorProx[6] > distancia || LeituraSensorProx[7] > distancia;

   
    
    if (obstaculo_frente) {
      motorEsquerdoVel = -1.0;
      motorDireitoVel = 1.0; 
    } else if (obstaculo_direita) {
      motorEsquerdoVel = 0.1;
      motorDireitoVel = 1.0;
    } else if (obstaculo_esquerda) {
      motorEsquerdoVel = 1.0;
      motorDireitoVel = 0.1; 
    } else {
      motorEsquerdoVel = 1.0;
      motorDireitoVel = 1.0; 
    }

    if (motorEsquerdoVel == -1.0 && motorDireitoVel == -1.0) {
      contadorVezesPreso++;
    } else {
      contadorVezesPreso = 0;
    }


    if (contadorVezesPreso > steps_stuck) {
      motorEsquerdoVel = 1.0;
      motorDireitoVel = -1.0; 
      contadorVezesPreso = 0; 
    }
    
    if ((posicaoRobo[0] - posicaoAnterior[0]) < respiro &&
        (posicaoRobo[1] - posicaoAnterior[1]) < respiro &&
        (posicaoRobo[2] - posicaoAnterior[2]) < respiro) {
      contadorParaForcarRobo++;
    } else {
      contadorParaForcarRobo = 0;
    }

    if (contadorParaForcarRobo > 20) {
      motorEsquerdoVel = 1.0;
      motorDireitoVel = -1.0;
      contadorParaForcarRobo = 0;
    }
    
    for (i = 0; i < 3; i++) {
      posicaoAnterior[i] = posicaoCaixa[i];
    }
    

    wb_motor_set_velocity(MotorEsquerdo, 6.28 * motorEsquerdoVel);
    wb_motor_set_velocity(MotorDireito, 6.28 * motorDireitoVel);
  }

  wb_robot_cleanup();
  return 0;
}