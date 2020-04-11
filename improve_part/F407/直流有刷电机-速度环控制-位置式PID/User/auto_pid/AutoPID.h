#ifndef AUTOPID_H
#define	AUTOPID_H

/* AutoPID - Ianick Noejovich
 *
 * Basado en modelo ajeno, refactorizado y adaptado para el Club de Rob�tica
 *
 * Esto es una red neuronal, un PID que se ajusta a si mismo. Actua
 * teniendo en cuenta al controlador PID como una red neuronal de
 * una capa, con tres inputs y un output. Cada epoch (iteracion) se
 * hace backpropagation para poder ajustar los pesos (las constantes)
 * del PID y as� obtener el error m�s cercano a 0 posible. Una vez
 * que el error est� dentro del threshold, se deja de tunear.
 *
 * En lo posible, este programa se debe instalar con un PID inicial que
 * funcione, ya que caso contrario el robot se ir�a de la pista antes de
 * poder aprender y auto-tunearse. Los pesos para el PID inicial se
 * pueden modificar en las lineas 28 a 32 de este mismo archivo.
 */


// CONSTANTES (TOCAR PARA CAMBIAR LA PRECISION) (ADAPTAR A CADA ROBOT) //
extern const int epochLength;     // CANTIDAD DE ITERACIONES POR EPOCH
extern const double errorThreshold; // THRESHOLD DE ERROR
extern const double learnRate;   // LEARNING RATE
/////////////////////////////////////////////////////////////////////////

// PESOS INICIALES //
extern const double Kp;
extern const double Ki;
extern const double Kd;

double PID(double err);
/////////////////////

#endif
