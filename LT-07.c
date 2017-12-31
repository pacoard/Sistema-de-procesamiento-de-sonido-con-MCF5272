//Francisco García de la Corte
//Carlos Santos Rancaño
#include "m5272.h"
#include "m5272lib.c"
#include "m5272adc_dac(mod).c"
#include "m5272gpio.c"

//--------------Interfaz---------------
typedef enum {INICIO=-1, MODULO=0, FASE=1, AFINADOR=2, ANALIZADOR_HW=3, FILTRO_SINTONIZABLE=4} ESTADO;
ESTADO estado = INICIO;


//Constantes para la interrupción ------------------------------------------------------------------------------
#define FONDO_ESCALA 0xFFF	// Valor de lectura mÃ¡xima del ADC
#define V_MAX 5	
#define V_BASE 0x40				  // DirecciÃ³n de inicio de los vectores de interrupciÃ³n
#define DIR_VTMR0 4*(V_BASE+5)			  // DirecciÃ³n del vector de TMR0
#define FREC_INT 4000				  // Frec. de interr. TMR0 = 4000 Hz (cada 0.25ms)
#define PRESCALADO 2					//parece que el preescalado no hace nada
#define CNT_INT1 MCF_CLK/(FREC_INT*PRESCALADO*16)	  // Valor de precarga del temporizador de interrupciones TRR0
#if CNT_INT1>0xFFFF
#error PRESCALADO demasiado pequeÃ±o para esa frecuencia (CNT_INT1>0xFFFF)
#endif
#define BORRA_REF 0x0002			  // Valor de borrado de interr. pendientes de tout1 para TER0
#define FONDO_ESCALA 0xFFF	// Valor de lectura mÃ¡xima del ADC
#define V_MAX 5	
volatile ULONG cont_retardo;

//Constantes y variables para el cálculo de la DFT y las mejoras ------------------------------------------------
#define NUM_MUESTRAS_PERIODO_5HZ 800 //nueva dimensión para el array de muestras de la sinusoide, ahora a 5Hz.
#define ESC_ADC 819
#define N 80
#define DESESCALADO 20 //Al cuadrado, en realidad es 10
//Orden de la DFT: 20 muestras de la TF
#define N_FRECS 20
//32 niveles para los umbrales
#define NIVELES 32

static int umbrales[NIVELES]={63095, 84924, 114304, 153849, 207075, 278715, 375140, 504923,
	679607, 914724, 1231182, 1657123, 2230422, 3002059 , 4040653 , 5438559 , 7320085,
	9852544 ,13261133, 17848959, 24023991, 32335339, 43522083, 58578997, 78845006, 
	106122250, 142836338, 192252044, 258763624, 348285572, 468778561, 686492401};

static int escalado[NIVELES]={122, 245, 368, 491, 614, 737, 860, 983, 1105, 1228,
	1351, 1474, 1597, 1720, 1843, 1966, 2088, 2211, 2334, 2457, 2580,
	2703, 2826, 2949, 3072, 3194, 3317, 3440, 3563, 3686, 3809, 3932};

static int sin5Hz[NUM_MUESTRAS_PERIODO_5HZ] = {0, 6, 13, 19, 26, 32, 39, 45, 51, 58, 64, 71, 77, 83, 90, 96, 103, 
	109, 115, 122, 128, 134, 141, 147, 153, 160, 166, 172, 179, 185, 191, 197, 204, 210, 
	216, 222, 228, 235, 241, 247, 253, 259, 265, 271, 277, 283, 289, 296, 301, 307, 313, 
	319, 325, 331, 337, 343, 349, 355, 360, 366, 372, 378, 383, 389, 395, 400, 406, 411, 
	417, 422, 428, 433, 439, 444, 450, 455, 460, 466, 471, 476, 481, 487, 492, 497, 502, 
	507, 512, 517, 522, 527, 532, 537, 542, 546, 551, 556, 561, 565, 570, 575, 579, 584, 
	588, 593, 597, 601, 606, 610, 614, 619, 623, 627, 631, 635, 639, 643, 647, 651, 655, 
	659, 663, 666, 670, 674, 677, 681, 685, 688, 692, 695, 698, 702, 705, 708, 711, 715, 
	718, 721, 724, 727, 730, 733, 735, 738, 741, 744, 746, 749, 752, 754, 757, 759, 761, 
	764, 766, 768, 771, 773, 775, 777, 779, 781, 783, 785, 786, 788, 790, 792, 793, 795, 
	796, 798, 799, 801, 802, 803, 804, 806, 807, 808, 809, 810, 811, 812, 813, 813, 814, 
	815, 815, 816, 816, 817, 817, 818, 818, 818, 819, 819, 819, 819, 819, 819, 819, 819, 
	819, 818, 818, 818, 817, 817, 816, 816, 815, 815, 814, 813, 813, 812, 811, 810, 809, 
	808, 807, 806, 804, 803, 802, 801, 799, 798, 796, 795, 793, 792, 790, 788, 786, 785, 
	783, 781, 779, 777, 775, 773, 771, 768, 766, 764, 761, 759, 757, 754, 752, 749, 746, 
	744, 741, 738, 735, 733, 730, 727, 724, 721, 718, 715, 711, 708, 705, 702, 698, 695, 
	692, 688, 685, 681, 677, 674, 670, 666, 663, 659, 655, 651, 647, 643, 639, 635, 631, 
	627, 623, 619, 614, 610, 606, 601, 597, 593, 588, 584, 579, 575, 570, 565, 561, 556, 
	551, 546, 542, 537, 532, 527, 522, 517, 512, 507, 502, 497, 492, 487, 481, 476, 471, 
	466, 460, 455, 450, 444, 439, 433, 428, 422, 417, 411, 406, 400, 395, 389, 383, 378, 
	372, 366, 360, 355, 349, 343, 337, 331, 325, 319, 313, 307, 301, 296, 289, 283, 277, 
	271, 265, 259, 253, 247, 241, 235, 228, 222, 216, 210, 204, 197, 191, 185, 179, 172, 
	166, 160, 153, 147, 141, 134, 128, 122, 115, 109, 103, 96, 90, 83, 77, 71, 64, 58, 51, 
	45, 39, 32, 26, 19, 13, 6, 0, -6, -13, -19, -26, -32, -39, -45, -51, -58, -64, -71, -77, 
	-83, -90, -96, -103, -109, -115, -122, -128, -134, -141, -147, -153, -160, -166, -172, 
	-179, -185, -191, -197, -204, -210, -216, -222, -228, -235, -241, -247, -253, -259, -265, 
	-271, -277, -283, -289, -296, -301, -307, -313, -319, -325, -331, -337, -343, -349, -355,
	-360, -366, -372, -378, -383, -389, -395, -400, -406, -411, -417, -422, -428, -433, -439, 
	-444, -450, -455, -460, -466, -471, -476, -481, -487, -492, -497, -502, -507, -512, -517, 
	-522, -527, -532, -537, -542, -546, -551, -556, -561, -565, -570, -575, -579, -584, -588, 
	-593, -597, -601, -606, -610, -614, -619, -623, -627, -631, -635, -639, -643, -647, -651, 
	-655, -659, -663, -666, -670, -674, -677, -681, -685, -688, -692, -695, -698, -702, -705, 
	-708, -711, -715, -718, -721, -724, -727, -730, -733, -735, -738, -741, -744, -746, -749, 
	-752, -754, -757, -759, -761, -764, -766, -768, -771, -773, -775, -777, -779, -781, -783, 
	-785, -786, -788, -790, -792, -793, -795, -796, -798, -799, -801, -802, -803, -804, -806, 
	-807, -808, -809, -810, -811, -812, -813, -813, -814, -815, -815, -816, -816, -817, -817, 
	-818, -818, -818, -819, -819, -819, -819, -819, -819, -819, -819, -819, -818, -818, -818, 
	-817, -817, -816, -816, -815, -815, -814, -813, -813, -812, -811, -810, -809, -808, -807, 
	-806, -804, -803, -802, -801, -799, -798, -796, -795, -793, -792, -790, -788, -786, -785, 
	-783, -781, -779, -777, -775, -773, -771, -768, -766, -764, -761, -759, -757, -754, -752, 
	-749, -746, -744, -741, -738, -735, -733, -730, -727, -724, -721, -718, -715, -711, -708, 
	-705, -702, -698, -695, -692, -688, -685, -681, -677, -674, -670, -666, -663, -659, -655, 
	-651, -647, -643, -639, -635, -631, -627, -623, -619, -614, -610, -606, -601, -597, -593, 
	-588, -584, -579, -575, -570, -565, -561, -556, -551, -546, -542, -537, -532, -527, -522, 
	-517, -512, -507, -502, -497, -492, -487, -481, -476, -471, -466, -460, -455, -450, -444, 
	-439, -433, -428, -422, -417, -411, -406, -400, -395, -389, -383, -378, -372, -366, -360, 
	-355, -349, -343, -337, -331, -325, -319, -313, -307, -301, -296, -289, -283, -277, -271, 
	-265, -259, -253, -247, -241, -235, -228, -222, -216, -210, -204, -197, -191, -185, -179, 
	-172, -166, -160, -153, -147, -141, -134, -128, -122, -115, -109, -103, -96, -90, -83, 
	-77, -71, -64, -58, -51, -45, -39, -32, -26, -19, -13, -6};

int muestra;

int dft[N_FRECS]; // = dftCos^2 + dftSin^2
int dftCos[N_FRECS]; //parte real
int dftSin[N_FRECS]; //parte imaginaria

int i, n, k, j; //iteradores para bucles
int cont160Inter;
int calculo; //Si calculo o no la DFT en función de si me quedan por sacar por pantalla

static int saltoPrincipal[N_FRECS] = {10, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240, 260, 280, 300, 320, 340, 360, 380};
int salto[N_FRECS];

int *pSin[N_FRECS];
int *pCos[N_FRECS];

//--------Analizador de circuitos---------
int indice = 0;
int cont1600ints = 0;
int barrido = 0; //contador de frecuencias: aumenta en 1 con cada dft calculada
int mostrarHw = 0; //a activar cuando se ha calculado
int Hw[N_FRECS];

int modulo_fase = 0; //si es uno, se visualizará la fase

//--------Sistema de filtrado digital y reconstrucción de la señal-------
int freqBaja=0;
int freqAlta=0;

int salidaAnalogica=0;

int DFTlista=0;
int nFiltro=0;
int coef[N_FRECS];
int sumaCoef=0;
int salida[N];

int salidaAnalogicaCalculada = 0;

int *indicesSenos[N_FRECS];

//------------Teclado------------
#define NUM_FILAS 4
#define NUM_COLS 4
#define EXCIT 1

UWORD pSal=0x0000;
char opcion;
char tecla;

//---------------LCD-------------
#include "m5272lcd(mod).c"

//-------------Afinador---------- 
int freqAntes=0;
int freqMax=0;
char *maximoString;
int picoMax=0;
int explorar = 0; //1 si voy de 100 en 100 (como siempre sin el afinador) y 2 para la búsqueda precisa
int contador1seg=0; //para que recalcule solo la frecuencia maxima cada 4000 interrupciones (25 dfts, contando las que se desprecian)

//----------Micrófono-----------
int microfono = 0;
//PROGRAMA------------------------------------------------------------------------------------------------------

//------------------------------------------------------
// char teclado(void)
//
// Descripción: 
// Función basada en el tutorial teclado.c. Excita el teclado en busca
// de respuestas para obtener las teclas pulsadas por el usuario.
// Devuelve un char correspondiente a la tecla pulsada.
//------------------------------------------------------
char teclado(void) {
  BYTE fila, columna, fila_mask;
  static char teclas[4][4] = {{"123C"},
                              {"456D"},
                              {"789E"},
                              {"A0BF"}};
  // Bucle de exploración del teclado
  while(TRUE){
    // Excitamos una columna
    for(columna = NUM_COLS - 1; columna >= 0; columna--){
      pSal = pSal & 0xFFF0;
      pSal= (EXCIT << columna) | pSal;
      set16_puertoS(pSal);		// Se envía la excitación de columna
      retardo(1150);				// Esperamos respuesta de optoacopladores
      // Exploramos las filas en busca de respuesta
      for(fila = NUM_FILAS - 1; fila >= 0; fila--){
        fila_mask = EXCIT << fila;		// Máscara para leer el bit de la fila actual
        if(lee16_puertoE() & fila_mask){		// Si encuentra tecla pulsada,
          while(lee16_puertoE() & fila_mask);	//   Esperamos a que se suelte
          retardo(1150);			//   Retardo antirrebotes
          return teclas[fila][columna];	//   Devolvemos la tecla pulsada
        }
      } // Siguiente columna
    }// Exploración finalizada sin encontrar una tecla pulsada 
  }// Reiniciamos exploración
}

//------------------------------------------------------
// void sacaLCD(char* sacar, int linea)
//
// Descripción: 
// Función basada en el tutorial lcd.c. Escribe por el LCD el array de
// chars pasado como parámetro, refrescando la pantalla y colocando el 
// cursor al principio. Además, se puede seleccionar en cuál de las dos
// líneas del LCD se desea escribir mediante el segundo parámetro.
//------------------------------------------------------
void sacaLCD(char* sacar, int linea) {
  
  if (linea==1) {
	LCD_inst(CLR_DISP);		// Limpiamos display
	LCD_inst(LIN_1LCD); // Movemos el cursor a la 1ª línea
	
  }		
  if (linea==2)
	LCD_inst(LIN_2LCD); // Movemos el cursor a la 2ª línea

  while(*sacar){  // Imprime "HOLA" en el display
    LCD_dato(*sacar++);	// carácter a carácter
  }
  //No se borrará el mensaje hasta que llamemos a esta función de nuevo
}

//------------------------------------------------------
// int strToInt (char str)
//
// Descripción: 
// Simple rutina auxiliar para nuestro sistema de filtrado digital.
// Se llama a esta función para pasar a enteros los chars intruducidos
// desde el teclado. 
// Devuelve un char equivalente al int pasado como parámetro
//------------------------------------------------------
int strToInt (char str) {
	switch(str) {
			case '0': return 0; 
			case '1': return 1; 
			case '2': return 2; 
			case '3': return 3; 
			case '4': return 4; 
			case '5': return 5; 
			case '6': return 6; 
			case '7': return 7; 
			case '8': return 8; 
			case '9': return 9;
			default: break;
		}
	return 0;
}

//------------------------------------------------------
// char * stringFrecuencia(int f)
//
// Descripción: 
// Esta función tiene como objetivo adaptar los números de frecuencias hallados
// por el afinador a un formato de array de chars "XXXX Hz", de forma que se
// pueda sacar por el LCD. Halla los millares, centenas, decenas y unidades del
// número, y los guarda en posiciones de memoria consecutivas seguidas de " Hz".
// Devuelve un puntero a la primera posición en memoria de estos chars, hallados
// a partir del número pasado como parámetro.
//------------------------------------------------------
char * stringFrecuencia(int f) { //Devuelve un puntero a un array de 7 chars
	//Generar string a sacar:
	static char freq[7];
	i=0;
	k=0;
	//Cada if a partir de ahora, obtiene el numero a la izquierda,
	//lo quita y deja los numeros de la derecha
	if (f>=1000) {
		freq[k] = '1';
		f -=1000;
		
	} else {
		freq[k] = ' ';
	}
	k++;

	if (f>=100) {
		while ((f-100) >= 0) {
			f-=100;
			i++;
		}
		switch(i) {
			case 0: freq[k] = '0'; break;
			case 1: freq[k] = '1'; break;
			case 2: freq[k] = '2'; break;
			case 3: freq[k] = '3'; break;
			case 4: freq[k] = '4'; break;
			case 5: freq[k] = '5'; break;
			case 6: freq[k] = '6'; break;
			case 7: freq[k] = '7'; break;
			case 8: freq[k] = '8'; break;
			case 9: freq[k] = '9'; break;
			default: break;
		}
		
	}
	if(i == 0 ){
		if (freq[k-1]==' '){
			freq[k] = ' ';
		} else {
			freq[k] = '0';
		}
	}
	i=0;
	k++;
	if (f>=10) {
		while ((f-10) >= 0) {
			f-=10;
			i++;
		}
		switch(i) {
			case 0: freq[k] = '0'; break;
			case 1: freq[k] = '1'; break;
			case 2: freq[k] = '2'; break;
			case 3: freq[k] = '3'; break;
			case 4: freq[k] = '4'; break;
			case 5: freq[k] = '5'; break;
			case 6: freq[k] = '6'; break;
			case 7: freq[k] = '7'; break;
			case 8: freq[k] = '8'; break;
			case 9: freq[k] = '9'; break;
			default: break;
		}
	}
	if(i == 0){
		if(freq[k-1] == ' '){
			freq[k] = ' ';
		} else{
			freq[k]='0';
		}
	}
	i=0;
	k++;
	switch(f) {
			case 0: freq[k] = '0'; break;
			case 1: freq[k] = '1'; break;
			case 2: freq[k] = '2'; break;
			case 3: freq[k] = '3'; break;
			case 4: freq[k] = '4'; break;
			case 5: freq[k] = '5'; break;
			case 6: freq[k] = '6'; break;
			case 7: freq[k] = '7'; break;
			case 8: freq[k] = '8'; break;
			case 9: freq[k] = '9'; break;
			default: break;
		}

	freq[k+1] = ' ';
	freq[k+2] = 'H';
	freq[k+3] = 'z';
	return freq;
}

//------------------------------------------------------
// void cambiaRangoExploracion(void)
//
// Descripción: 
// Gestiona el cambio de ventana de análisis en frecuencia, de acuerdo con el pico
// máximo detectado en el cálculo de la DFT. Controla los casos límites y saca por 
// el LCD la frecuencia hallada
//------------------------------------------------------
void cambiaRangoExploracion(void) {
	cli(); //Parar la interrupción para los siguientes cálculos costosos
	explorar++;
	switch (explorar) {
		case 1: //Nuevo rango de exploración: ventana de 100Hz y paso de 5Hz
			if (freqMax<=55) 
				freqAntes = 5;
			else if (freqMax>=1945) 
				freqAntes = 1895;
			//Si no entramos en ningún caso limite
			else freqAntes = freqMax-50;

			for (i=0; i<N_FRECS; i++) { //con sumar uno a cada salto, se aumenta la frec en 5Hz
				salto[i] = (freqAntes/5) + i;
				//Nunca se van a pasar de las 800 muestras del seno
				pSin[i] = sin5Hz + salto[0];
				pCos[i] = sin5Hz + salto[0] + (NUM_MUESTRAS_PERIODO_5HZ>>2);
			}
			picoMax=0;
		break;
		
		case 2://Vuelta al calculo de la DFT original
			for(i=0; i<N_FRECS; i++) {
				salto[i] = saltoPrincipal[i];
			}
			maximoString = stringFrecuencia(freqMax);
			sacaLCD(maximoString, 2);
	
			explorar=0;
			picoMax=0;
			freqAntes=0;;
			freqMax=0;
			
			contador1seg=0;
		break;
		default: break;
	}
	sti(); //habilitar interrupciones
}

//------------------------------------------------------
// int * estimaFase0a90(int re, int im)
//
// Descripción: 
// Función auxiliar para el cálculo de la fase. Una vez se ha detectado
// en qué cuadrante se encuentra la fase, se llama a esta función para 
// que de un valor más preciso dentro de ese cuadrante. Se sigue la
// estrategia "divide y vencerás".
// Devuelve un puntero a un valor del array de escalado, como representación
// de los grados como un nivel de tensión en el osciloscopio
//------------------------------------------------------
int * estimaFase0a90(int re, int im) {
	int *pfase0a90;
	if (re<0) re*=-1;
	if (im<0) im*=-1;

	if (re>im) {//0 a 45
		//22.5 cuando aprox. cuando Im = 0.414*Re
		// o aprox 10*Im = 4*Re => 21.8
		if (10*im < 4*re) {//0 a 22.5 (21.8)
			//11.25 cuando Im = 0.19*Re
			// o 10*Im = 2*Re
			if (10*im < 2*re) //0 a 11.25 (11.3)
				pfase0a90 = escalado; //0
			else //11.25 a 22.5
				pfase0a90 = escalado+1; //11.25
			}
		else {//22.5 a 45
			//33.75 si Im = 0.668*Re
			if (10*im < 7*re) //22.5 a 33.75 (35)
				pfase0a90 = escalado+2; //22.5
			else //33.75 a 45
				pfase0a90 = escalado+3; //33.75
		}
	}
	else {//45 a 90
		//22.5 cuando aprox. cuando Im = 0.414*Re
		// o aprox 10*Im = 4*Re => 21.8
		if (10*re > 4*im) {//45 a 67.5
			if (10*re > 2*im) //45 a 56.25
				pfase0a90 = escalado+4; //45
			else //56.25 a 67.5
				pfase0a90 = escalado+5; //56.25
			}
		else {//67.5 a 90
			if (10*re > 7*im) //67.5 a 78.75
				pfase0a90 = escalado+6; //67.5
			else //78.75 a 90
				pfase0a90 = escalado+7; //78.75
		}
	}
	return pfase0a90;
}

//------------------------------------------------------
// int conversion_dB(int x)
//
// Descripción: 
// Convierte los valores calculados de la DFT a decibelios
// de una forma eficiente al seguir la estrategia "divide 
// y vencerás".
// Devuelve un valor de el array de escalado del DAC
//------------------------------------------------------
int conversion_dB(int x) {
    if (x>umbrales[31]) return escalado[31];
    if (x<umbrales[0]) return escalado[0];
 
    if (x>umbrales[16]) { // 16 - 31
        if (x>umbrales[24]) { // 24 - 31
            if (x>umbrales[28]) { // 28 -31
                if (x>umbrales[30]) { // 30 - 31
                    if (x>umbrales[31]) return escalado[31];
                    else return escalado[30];
                }
                else { // 28 - 29
                    if (x>umbrales[29]) return escalado[29];
                    else return escalado[28];
                }
            }
            else { // 16 - 19
                if (x>umbrales[26]) { // 26 - 27
                    if (x>umbrales[27]) return escalado[27];
                    else return escalado[26];
                }
                else { // 24 - 25
                    if (x>umbrales[25]) return escalado[25];
                    else return escalado[24];
                }
            }
        }
        else { // 16 - 23
            if (x>umbrales[20]) { // 20 - 23
                if (x>umbrales[22]) { // 22 - 23
                    if (x>umbrales[23]) return escalado[23];
                    else return escalado[22];
                }
                else { // 20 - 21
                    if (x>umbrales[21]) return escalado[21];
                    else return escalado[20];
                }
            }
            else { // 16 - 19
                if (x>umbrales[18]) { // 18 - 19
                    if (x>umbrales[19]) return escalado[19];
                    else return escalado[18];
                }
                else { // 16 - 17
                    if (x>umbrales[17]) return escalado[17];
                    else return escalado[16];
                }
            }
        }
    }
 
    else{ // 0 - 15
        if (x>umbrales[8]) { // 8 - 15
            if (x>umbrales[12]) { // 12 - 15
                if (x>umbrales[14]) { // 14 - 15
                    if (x>umbrales[15]) return escalado[15];
                    else return escalado[14];
                }
                else { // 12 - 13
                    if (x>umbrales[13]) return escalado[13];
                    else return escalado[12];
                }
            }
            else { // 8 - 11
                if (x>umbrales[10]) { // 10 -11
                    if (x>umbrales[11]) return escalado[11];
                    else return escalado[10];
                }
                else { // 8 - 9
                    if (x>umbrales[9]) return escalado[9];
                    else return escalado[8];
                }
            }
        }
        else { // 0 - 7
            if (x>umbrales[4]) { // 4 - 7
                if (x>umbrales[6]) { // 6 - 7
                    if (x>umbrales[7]) return escalado[7];
                    else return escalado[6];
                }
                else { // 4 - 5
                    if (x>umbrales[5]) return escalado[5];
                    else return escalado[4];
                }
            }
            else { // 0 - 3
                if (x>umbrales[2]) { // 2 - 3
                    if (x>umbrales[3]) return escalado[3];
                    else return escalado[2];
                }
                else { // 0 - 1
                    if (x>umbrales[1]) return escalado[1];
                    else return escalado[0];
                }
            }   
        }
    }
    return 0;
}

//------------------------------------------------------
// void DFT_Inversa(void)
//
// Descripción: 
// Esta rutina reconstruye la señal que pasa por el filtro sintonizable,
// es decir, genera en el tiempo la señal filtrada. Calcula los pesos
// de cada componente en frecuencia y devuelve una suma de senos
// ponderados por esos pesos, para posteriormente sacar esta suma por
// el DAC. Es necesario parar la interrupción para estos cálculos.
//------------------------------------------------------
void DFT_Inversa(void) {
	cli(); //Parar interrupción para el cálculo de la DFT inversa
	//Cálculo de coeficientes para ponderar los senos
	sumaCoef=0;
	for(i=0; i<N_FRECS; i++) {
		coef[i] = dft[i]/125; //dft ya en decibelios
		sumaCoef+= coef[i];
	}
	
	sumaCoef++;
	//Cálculo de las muestras de salida
	for (i=0; i<N; i++){
		for(j=0; j<N_FRECS; j++) {

			salida[i]+= coef[j]*(*indicesSenos[j] + 2500);
			indicesSenos[j] += salto[j];

			if((indicesSenos[j] - sin5Hz) >= NUM_MUESTRAS_PERIODO_5HZ) 
				indicesSenos[j] -= NUM_MUESTRAS_PERIODO_5HZ;
		}
		salida[i] = salida[i]/sumaCoef;
	}
  	sti(); //habilitar interrupciones
}

//------------------------------------------------------
// void rutina_tout0(void)
//
// Descripción: 
// Interrupción que originalmente sólo muestraba la señal de entrada
// y calculaba su DFT. Ahora se ocupa de todas las mejoras, 
// haciendo las operaciones pertinentes de acuerdo con la mejora
// seleccionada.
//------------------------------------------------------
void rutina_tout0(void) { 
    
	mbar_writeShort(MCFSIM_TER0,BORRA_REF); 	// Reset del bit de fin de cuenta

	n++;
	//Obtenemos muestra
	muestra = ADC_dato();
	
	if(estado == ANALIZADOR_HW) {
		if (mostrarHw) //Ya calculada H(w), la sacamos
			DAC_dato(Hw[cont160Inter>>3]);
		else { //Mientras se esté calculando, se barre en frecuencia
			DAC_dato(sin5Hz[indice] + 2500);
			barrido = cont1600ints/80; //misma frecuencia durante 80 interrupciones
			indice = (indice + salto[barrido])%NUM_MUESTRAS_PERIODO_5HZ; 
			
			cont1600ints++;
			if(cont1600ints == 1600) { //Cuando se termina de calcular H(W)
				cont1600ints = 0;
				mostrarHw = 1;
				barrido = 0;
			}
		}	
	}
	else if(salidaAnalogica && DFTlista) { //para el filtro sintonizable
		DAC_dato(salida[nFiltro]);
		nFiltro++;
		if (nFiltro == N) nFiltro=0;
	}
	else { //para el resto de los estados
		DAC_dato(dft[cont160Inter>>3]);
	}
	
	cont160Inter++;
	
	if(cont160Inter == 160) {
		for (k=0; k<N_FRECS; k++) {
			dftCos[k] = 0;
			dftSin[k] = 0;
			pSin[k] = sin5Hz;
			pCos[k] = sin5Hz + (NUM_MUESTRAS_PERIODO_5HZ>>2); //empieza en sin5Hz[200]
		}
		calculo = 1;  //Vuelvo a calcular dft al sacar las 20 componentes (calculo = 1)	
		cont160Inter = 0;
		n = 0;
		pSal = pSal | 0x0010;
		set16_puertoS(pSal);//activar rampa
	}
	else {
		pSal = pSal & 0xFFEF;
		set16_puertoS(pSal);
	}
	
	//Calcular aportaciones real e imaginaria de la muestra
	for (k=0; k<N_FRECS; k++) {		
		dftCos[k] += muestra*(*pCos[k]);	
		dftSin[k] += muestra*(*pSin[k]);
		//Actualizamos los Índices
		pSin[k] += salto[k];
		pCos[k] += salto[k];
		//Buffer circular
		if((pSin[k] - sin5Hz) >= NUM_MUESTRAS_PERIODO_5HZ) 
			pSin[k] -= NUM_MUESTRAS_PERIODO_5HZ;
		if((pCos[k] - sin5Hz) >= NUM_MUESTRAS_PERIODO_5HZ) 
			pCos[k] -= NUM_MUESTRAS_PERIODO_5HZ;
	}
	
	//Calculo del modulo de la dft		
	if(calculo && (n == N)){
		//Calculo del mÃ³dulo al cuadrado
		for (k=0; k<N_FRECS; k++) {
			switch(estado) {
			
			case MODULO:
				dft[k] = ((dftCos[k]>>DESESCALADO) * (dftCos[k])) 
						+ ((dftSin[k]>>DESESCALADO) * (dftSin[k]));
				dft[k] = conversion_dB(dft[k]);
			break;
			
			case FASE:
				if (dftCos[k]>0 && dftSin[k]>0) //0 a 90
					dft[k] = *estimaFase0a90(dftCos[k],dftSin[k]);
				else if (dftCos[k]<0 && dftSin[k]>0) //90 a 180
					dft[k] = *(estimaFase0a90(dftCos[k],dftSin[k])+8); //+90
				else if (dftCos[k]<0 && dftSin[k]<0) //180 a 270
					dft[k] = *(estimaFase0a90(dftCos[k],dftSin[k])+16); //+180
				else /*if (dftCos[k] > 0 && dftSin[k] < 0)*/ //270 a 360
					dft[k] = *(estimaFase0a90(dftCos[k],dftSin[k])+24); //+270
			break;
			
			case AFINADOR:
				dft[k] = ((dftCos[k]>>DESESCALADO) * (dftCos[k])) 
						+ ((dftSin[k]>>DESESCALADO) * (dftSin[k]));
			
				if (dft[k] > picoMax) {
					picoMax= dft[k];
					freqMax = salto[k]*5;
				}
				dft[k] = conversion_dB(dft[k]);
			break;

			case ANALIZADOR_HW:
				if (!modulo_fase) { //Calcular módulo
					dft[k] = ((dftCos[k]>>DESESCALADO) * (dftCos[k])) 
							+ ((dftSin[k]>>DESESCALADO) * (dftSin[k]));
					dft[k] = conversion_dB(dft[k]);
				}
				else { //Calcular fase
					if (dftCos[k]>0 && dftSin[k]>0) //0 a 90
						dft[k] = *estimaFase0a90(dftCos[k],dftSin[k]);
					else if (dftCos[k]<0 && dftSin[k]>0) //90 a 180
						dft[k] = *(estimaFase0a90(dftCos[k],dftSin[k])+8); //+90
					else if (dftCos[k]<0 && dftSin[k]<0) //180 a 270
						dft[k] = *(estimaFase0a90(dftCos[k],dftSin[k])+16); //+180
					else /*if (dftCos[k] > 0 && dftSin[k] < 0)*/ //270 a 360
						dft[k] = *(estimaFase0a90(dftCos[k],dftSin[k])+24); //+270
				}
			break;

			case FILTRO_SINTONIZABLE:
				
					if (k<=freqBaja/100 || k>=freqAlta/100) 
						dft[k]=escalado[0];
					else {
						dft[k] = ((dftCos[k]>>DESESCALADO) * (dftCos[k])) 
								+ ((dftSin[k]>>DESESCALADO) * (dftSin[k]));
						dft[k] = conversion_dB(dft[k]);
					}	
				
			break;
			
			default: break;
			}
			if (estado==ANALIZADOR_HW) Hw[barrido] = dft[barrido];
			//Reiniciamos acumuladores
			dftCos[k] = 0;
			dftSin[k] = 0;
			//Reiniciamos indices a seno y coseno
			pSin[k] = sin5Hz;
			pCos[k] = sin5Hz + (NUM_MUESTRAS_PERIODO_5HZ>>2); //empieza en sin5Hz[200]
		}
		calculo = 0;
		n=0;
		if (estado==AFINADOR) {
			contador1seg++;
			if (contador1seg >= 25) {
				cambiaRangoExploracion();
			}	
		}
		if (estado==FILTRO_SINTONIZABLE) {
			DFTlista = 1;
			if (salidaAnalogica && !salidaAnalogicaCalculada) {
				DFT_Inversa();
				salidaAnalogicaCalculada=1;
			}
		}
	}
}

//------------------------------------------------------
// void __init(void)
//
// Descripción: 
// Rutina de inicialización del programa. Valor inicial de
// variables, punteros, inicialización de la interrupción,
// DAC/ADC y LCD.
//------------------------------------------------------
void __init(void) { 		
	calculo = 1;
	n = 0;
	cont160Inter = 0;

	for (i=0; i<N_FRECS; i++) {
  		dft[i] = escalado[0]; //inicializado al valor que DAC reconoce como cero
		Hw[i] = escalado[0];

		dftSin[i] = 0;
  		dftCos[i] = 0;
		salto[i] = saltoPrincipal[i];
		pSin[i] = sin5Hz;
		pCos[i] = sin5Hz + (NUM_MUESTRAS_PERIODO_5HZ>>2); //empieza en sin5Hz[200]

		indicesSenos[i] = sin5Hz;
	}
	
  LCD_reset();	// Reseteamos el LCD
  LCD_init();	// e inicializamos el display

  DAC_ADC_init();
  mbar_writeByte(MCFSIM_PIVR,V_BASE);			// Fija comienzo de vectores de interrupción en V_BASE.
  ACCESO_A_MEMORIA_LONG(DIR_VTMR0)= (ULONG)_prep_TOUT0; // Escribimos la direcciÃ³n de la función para TMR0
  output("COMIENZA EL PROGRAMA\r\n");
  mbar_writeShort(MCFSIM_TMR0, (PRESCALADO-1)<<8|0x3D);		// TMR0: PS=1-1=0 CE=00 OM=1 ORI=1 FRR=1 CLK=10 RST=1
  mbar_writeShort(MCFSIM_TCN0, 0x0000);		// Ponemos a 0 el contador del TIMER0
  mbar_writeShort(MCFSIM_TRR0, CNT_INT1);	// Fijamos la cuenta final del contador
  mbar_writeLong(MCFSIM_ICR1, 0x8888C888);	// Marca la interrupción del TIMER0 como no pendiente y de nivel 4
  sti();	// Habilitamos interrupciones
}

//------------------------------------------------------
// void bucleMain(void)
//
// Descripción: 
// Bucle principal del programa. Gestiona la interfaz de usuario,
// leyendo el teclado y actuando en consecuencia al elegir
// menús o submenús. Conmuta entre mejoras según las opciones
// elegidas por el usuario a través del teclado matricial.
//------------------------------------------------------
void bucleMain(void) {
	
	output("Pulse una de las siguientes opciones\n");
    output("1) Modulo\n");
    output("2) Fase\n");
    output("3) Afinador\n");
    output("4) Analizador de espectros\n");
	output("5) Filtro sintonizable\n");
	output("6) Activar/desactivar micrófono\n");
    output("====================\n");
	
	while(1){
		opcion = teclado();

	    switch (opcion){
	      case '1': output("Has elegido modulo\n");
	        		estado = MODULO;
	   				sacaLCD("MODULO",1);
				break;
	      case '2': output("Has elegido fase\n");
	        		estado = FASE;
	        		sacaLCD("FASE",1);
				break;
	     
	      case '3': output("Has elegido afinador\n");
	        		estado = AFINADOR;
					sacaLCD("AFINADOR", 1);
	        		//sacaremos el maximo por el LCD una vez se calcule
				break;
	      case '4': output("Has elegido analizador de circuitos\n");
		  			if (estado!=ANALIZADOR_HW) 
		  				estado = ANALIZADOR_HW;
	        		else mostrarHw = 0; //para que recalcule la Hw si se ha pulsado de nuevo
					indice=0;
	        		
	        		sacaLCD("ANALIZADOR",1);
					sacaLCD("DE ESPECTROS",2);
					output("\nElija lo que quiera visualizar: \n");
					output("1) Modulo\n");
    				output("2) Fase\n");
    				output("F) Volver al menú principal\n");
    				
    				while(1) {
    					opcion = teclado();
    					if (opcion=='1') {
    						modulo_fase = 0;
    						mostrarHw = 0;
					}
    					
    					if (opcion=='2') {
    						modulo_fase = 1;
    						mostrarHw = 0;
					}
    					if (opcion == 'F') {
    						modulo_fase=0;
    						mostrarHw=0;
    						break;
    					}
    				}

				break;
	      case '5': output("Has elegido filtro sintonizable\n");
	        		estado = FILTRO_SINTONIZABLE;
	   				sacaLCD("FILTRO",1);
					sacaLCD("SINTONIZABLE",2);
					output("\nElija el tipo de filtro: \n");
				    output("1) Paso bajo\n");
				    output("2) Paso alto\n");
				    output("3) Paso banda\n");
				    output("F) Volver al menú principal\n");
					
					indice=0;
					freqAlta = 0;
					freqBaja = 0;

					while(1){
						
							opcion = teclado();

							switch (opcion){
							  case '1': output("\nHas elegido Paso bajo\n");
								  freqAlta = 0;
								  freqBaja = 0;
	        							sacaLCD("FILTRO",1);
	   									sacaLCD("PASO BAJO",2);
										output("Introduce la frecuencia de corte\n");

										opcion=teclado();
										freqAlta += strToInt(opcion)*1000;
										opcion=teclado();
										freqAlta += strToInt(opcion)*100;
										opcion=teclado();
										freqAlta += strToInt(opcion)*10;
										opcion=teclado();
										freqAlta += strToInt(opcion);
										output("FreqAlta = ");
										outNum(10, freqAlta, 0);
										output("\n");

								break;
							  case '2': output("\nHas elegido Paso alto\n"); 
								  freqAlta = 2000;
							      freqBaja = 0;
	        							
										sacaLCD("FILTRO",1);
	        							sacaLCD("PASO ALTO",2);
										output("Introduce la frecuencia de corte\n");

										opcion=teclado();
										freqBaja += strToInt(opcion)*1000;
										opcion=teclado();
										freqBaja += strToInt(opcion)*100;
										opcion=teclado();
										freqBaja += strToInt(opcion)*10;
										opcion=teclado();
										freqBaja += strToInt(opcion);
										output("FreqBaja = ");
										outNum(10, freqBaja, 0);
										output("\n");
								break;
						     
							  case '3': output("\nHas elegido Paso banda\n");
								  freqAlta = 0;
								  freqBaja = 0;
	        							
										sacaLCD("FILTRO",1);
										sacaLCD("PASO BANDA", 2);
										output("Introduce la frecuencia de corte Baja\n");
										opcion=teclado();
										freqBaja += strToInt(opcion)*1000;
										opcion=teclado();
										freqBaja += strToInt(opcion)*100;
										opcion=teclado();
										freqBaja += strToInt(opcion)*10;
										opcion=teclado();
										freqBaja += strToInt(opcion);
										output("FreqBaja = ");
										outNum(10, freqBaja, 0);
										output("\n");

										output("Introduce la frecuencia de corte Alta\n");
										opcion=teclado();
										freqAlta += strToInt(opcion)*1000;
										opcion=teclado();
										freqAlta += strToInt(opcion)*100;
										opcion=teclado();
										freqAlta += strToInt(opcion)*10;
										opcion=teclado();
										freqAlta += strToInt(opcion);
										output("FreqAlta = ");
										outNum(10, freqAlta, 0);
										output("\n");
	        							
								break;
										
							  default: break;

							}

							while(1){
								output("\nElija lo que quiere que se muestre: \n");
								output("1) DFT\n");
								output("2) Tiempo\n");
								output("F) Volver al menú principal\n");
								opcion = teclado();
								if(opcion=='1')
									salidaAnalogica = 0;
								if (opcion == '2') {
									salidaAnalogica = 1;
									salidaAnalogicaCalculada = 0;
								}

								if (opcion == 'F') break;
							}


							if (opcion == 'F') {
								salidaAnalogica = 0;	
								break;
							}
					}
					break;
	      case '6': 
				if (microfono) {//desactivar
					output("Micrófono desactivado\n");
					microfono = 0;
					pSal = pSal & 0xFFCF;
					set16_puertoS(pSal);
				}
				else {//activar
					output("Micrófono activado\n");
					microfono = 1;
					pSal = pSal | 0x0020;
					set16_puertoS(pSal);	 
				}
				break;
	      default: 
			output("Elije mejora a probar\n");
	    }
	}
}
//---------------------------------------------------------
// Definición de rutinas de atención a las interrupciones
// Es necesario definirlas aunque estén vacías
void rutina_int1(void){}
void rutina_int2(void){}
void rutina_int3(void){}
void rutina_int4(void){}
void rutina_tout1(void){}
void rutina_tout2(void){}
void rutina_tout3(void){}
