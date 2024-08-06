//* @version "v1.7"
/* Includes ---------------------------------------------------------------- */
#include <Lumos_inferencing.h>  // Incluye la biblioteca de inferencia Lumos
#include "freertos/FreeRTOS.h"  // Incluye las bibliotecas FreeRTOS necesarias
#include "freertos/task.h"
#include "driver/adc.h"         // Incluye la biblioteca para el control del ADC
#include "esp_adc_cal.h"       // Incluye la biblioteca para la calibración del ADC

/** Audio buffers, pointers and selectors */
typedef struct {
    int16_t *buffer;           // Buffer para almacenar muestras de audio
    uint8_t buf_ready;         // Indicador de buffer listo para inferencia
    uint32_t buf_count;        // Contador de muestras en el buffer
    uint32_t n_samples;        // Número de muestras por ventana de inferencia
} inference_t;

static inference_t inference;  // Instancia de la estructura de inferencia
static const uint32_t sample_buffer_size = 2048;  // Tamaño del buffer de muestras
static signed short sampleBuffer[sample_buffer_size];  // Buffer para almacenar las muestras de audio
static bool debug_nn = false;  // Cambiar a true para imprimir características generadas
static bool record_status = true;  // Estado de grabación de muestras de audio

/**
 * @brief Configuración del ADC
 */
void adc_init() {
    // Configurar el ADC1 en el canal 6 (GPIO34) con 12 bits de precisión y atenuación de 12 dB
    /* @note  ADC1: 8 channels: GPIO32 - GPIO39ADC1 channel GPIO pin  ADC2 channel  GPIO pin
     *        ADC2: 10 channels: GPIO0, GPIO2, GPIO4, GPIO12 - GPIO15, GOIO25 - GPIO27
     *        ADC1_CHANNEL_0  GPIO36  ADC2_CHANNEL_0  GPIO4
     *        ADC1_CHANNEL_1  GPIO37  ADC2_CHANNEL_1  GPIO0
     *        ADC1_CHANNEL_2  GPIO38  ADC2_CHANNEL_2  GPIO2
     *        ADC1_CHANNEL_3  GPIO39  ADC2_CHANNEL_3  GPIO15
     *        ADC1_CHANNEL_4  GPIO32  ADC2_CHANNEL_4  GPIO13
     *        ADC1_CHANNEL_5  GPIO33  ADC2_CHANNEL_5  GPIO12
     *        ADC1_CHANNEL_6  GPIO34  ADC2_CHANNEL_6  GPIO14
     *        ADC1_CHANNEL_7  GPIO35  ADC2_CHANNEL_7  GPIO27
     *        ADC2_CHANNEL_8  GPIO25
     *        ADC2_CHANNEL_9  GPIO26
     *
     *        Dado que el módulo ADC2 también es utilizado por el Wi-Fi, solo uno de ellos podría obtener la prioridad al usarlos simultáneamente. 
     *        Esto significa que la función adc2_get_raw() podría quedar bloqueada hasta que el Wi-Fi deje de usarse, y viceversa.
    */
    adc1_config_width(ADC_WIDTH_BIT_12); //ajusta la resolución del ADC a 12 bits, el maximo soportado por el ESP32
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_12); 
    /* @note El Voltaje de Referencia de Atenuación (Vref) para los ADC del ESP32 es un voltaje de referencia interno 
     *  utilizado en la medición de voltajes de entrada. Los ADC del ESP32 pueden aceptar voltajes analógicos que van desde 0 V hasta Vref.
     *  El valor de Vref puede variar entre diferentes chips, con un valor medio de aproximadamente 1.1 V. Para medir voltajes mayores que Vref,
     *  es necesario atenuar los voltajes de entrada antes de introducirlos en los ADC. Los ADC del ESP32 ofrecen cuatro opciones de atenuación,
     *  con una atenuación más alta que permite la medición de voltajes de entrada más altos.      
     *  Attenuation Measurable input voltage range 
     *  ADC_ATTEN_DB_0    100 mV ~ 950 mV
     *  ADC_ATTEN_DB_2_5  100 mV ~ 1250 mV
     *  ADC_ATTEN_DB_6    150 mV ~ 1750 mV
     *  ADC_ATTEN_DB_11   150 mV ~ 2450 mV
    */
}

/**
 * @brief Configuración inicial de Arduino
 */
void setup() {
    Serial.begin(115200);  // Iniciar comunicación serial a 115200 baudios
    while (!Serial);  // Esperar hasta que se establezca la conexión serial
    //Serial.println("Demostración de Inferencia con Edge Impulse");

    // Configurar ADC
    adc_init();

    // Imprimir resumen de configuraciones de inferencia
    ei_printf("Configuraciones de inferencia:\n");
    ei_printf("\tIntervalo: ");
    ei_printf_float((float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf(" ms.\n");
    ei_printf("\tTamaño del cuadro: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tDuración de la muestra: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
    ei_printf("\tNúmero de clases: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));

    ei_printf("\nIniciando inferencia continua en 2 segundos...\n");
    ei_sleep(2000);  // Esperar 2 segundos antes de iniciar la inferencia

    // Iniciar la captura de muestras de audio
    if (microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT) == false) {
        ei_printf("ERROR: No se pudo asignar el búfer de audio (tamaño %d), esto podría deberse a la longitud de la ventana de su modelo\r\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
        return;
    }

    ei_printf("Grabando...\n");  // Indicar que se está grabando audio
}

/**
 * @brief se presenta una implementación para realizar la clasificación de audio utilizando Edge Impulse en un dispositivo Arduino. El código se encarga de grabar 	audio a través de un micrófono, ejecutar el clasificador de Edge Impulse y mostrar las predicciones resultantes.
 * @note Conceptos Clave
		Microphone Inference Record: Función para grabar audio a través del micrófono.
		Run Classifier: Función que ejecuta el clasificador de Edge Impulse.
		Impresión de Resultados: Muestra las predicciones del clasificador y el puntaje de anomalía si está disponible.
		Activación de Pin: Activa o desactiva un pin GPIO según el resultado de la clasificación.
		Estructura del Código
		El código comienza grabando audio a través de la función microphone_inference_record(). Luego, se crea una señal de audio y se ejecuta el clasificador de Edge Impulse con esta señal. Posteriormente, se imprimen las predicciones del clasificador y se activa o desactiva un pin GPIO según el resultado de la clasificación. Finalmente, se resetea el watchdog en cada ciclo de loop.
 */
void loop() {
    bool m = microphone_inference_record(); // Realizar grabación de audio

    if (!m) {
        ei_printf("ERROR: Falló la grabación de audio...\n"); // Imprimir mensaje de error si la grabación falla
        return;
    }

    signal_t signal;
    signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
    signal.get_data = &microphone_audio_signal_get_data;

    ei_impulse_result_t result = { 0 };

    // Ejecutar el clasificador de Edge Impulse
    EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
    if (r != EI_IMPULSE_OK) {
        ei_printf("ERROR: Falló la ejecución del clasificador (%d)\n", r); // Imprimir mensaje de error si la ejecución del clasificador falla
        return;
    }

    // Imprimir las predicciones del clasificador
    ei_printf("Predicciones ");
    ei_printf("(DSP: %d ms., Clasificación: %d ms., Anomalía: %d ms.)",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(": \n");

    // Comentar el bucle de predicciones del clasificador (comentado por razones de depuración)
    /*for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: ", result.classification[ix].label);
        ei_printf_float(result.classification[ix].value);
        ei_printf("\n");
    }*/

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    puntaje de anomalía: ");
    ei_printf_float(result.anomaly);
    ei_printf("\n");
#endif

    digitalWrite(33, LOW);   // Desactivar el pin 33 (GPIO33)

    // Activa el pin 33 si la inferencia es mayor que 0.40000
    if (result.classification[0].value > 0.40000) {
        ei_printf("    %s: ", result.classification[0].label);
        ei_printf_float(result.classification[0].value);
        digitalWrite(33, HIGH);  // Activar el pin 33 (GPIO33)
    }

    // Resetear el watchdog en cada ciclo de loop
    vTaskDelay(pdMS_TO_TICKS(5));;
}


/**
 * @brief se presenta una función de callback para la inferencia de audio en un entorno de desarrollo Arduino. La función se encarga de procesar las muestras de audio recibidas y preparar el búfer para la inferencia.
	@note Conceptos Clave
	Callback: Función que se llama en respuesta a un evento específico, en este caso, la llegada de muestras de audio.
	Búfer de Inferencia: Almacena las muestras de audio necesarias para realizar la inferencia.
	n_samples: Número de muestras de audio a procesar en cada iteración.
	Estructura del Código
	La función audio_inference_callback recibe un parámetro n_samples, que representa la cantidad de muestras de audio a procesar. Luego, itera sobre cada muestra recibida y la agrega al búfer de inferencia. Cuando se alcanza la cantidad necesaria de muestras para la inferencia, se reinicia el contador del búfer y se marca como listo para la inferencia.
 */
static void audio_inference_callback(uint32_t n_samples) {
    for(int i = 0; i < n_samples; i++) {
        inference.buffer[inference.buf_count++] = sampleBuffer[i]; // Se añade cada muestra al búfer de inferencia

        if(inference.buf_count >= inference.n_samples) { // Si se alcanza el número de muestras necesario para la inferencia
          inference.buf_count = 0; // Se reinicia el contador del búfer
          inference.buf_ready = 1; // Se indica que el búfer está listo para la inferencia
        }
    }
}


/**
 * @brief Tarea para capturar muestras de audio del ADCEn el fragmento de código proporcionado, se presenta una función en Arduino c++ que se encarga de capturar muestras de audio de un canal específico de un convertidor analógico-digital (ADC). La función opera de manera continua mientras el estado de grabación esté activo, leyendo un número definido de muestras y realizando una llamada a una función de inferencia de audio.
 * @note Conceptos Clave
			samples_to_read: Cantidad de muestras a leer, definida como el argumento recibido por la función.
			sampleBuffer: Arreglo donde se almacenan las muestras de audio.
			adc1_get_raw(ADC1_CHANNEL_6): Función que obtiene la muestra del canal ADC1_CHANNEL_6.
			audio_inference_callback(samples_to_read): Función de inferencia de audio que se llama después de leer las muestras.
			vTaskDelay(pdMS_TO_TICKS(5)): Pausa de 5 milisegundos entre lecturas de muestras.
			vTaskDelete(NULL): Eliminación de la tarea actual al finalizar la captura de muestras.
		Estructura del Código
			La función capture_samples recibe como argumento la cantidad de muestras a leer. Dentro de un bucle while, se lee el número especificado de muestras del canal ADC1_CHANNEL_6 y se almacenan en sampleBuffer. Luego, se verifica el estado de grabación y se llama a la función de inferencia de audio si la grabación está activa. Finalmente, se realiza una pausa de 10 milisegundos antes de repetir el proceso. Una vez que el estado de grabación se desactiva, la tarea se elimina.	
 */
static void capture_samples(void* arg) {
    const uint32_t samples_to_read = (uint32_t)arg; // Se define la cantidad de muestras a leer como el argumento recibido.

    while (record_status) { // Mientras el estado de grabación esté activo.
        for (uint32_t i = 0; i < samples_to_read; i++) { // Bucle para leer las muestras.
            sampleBuffer[i] = adc1_get_raw(ADC1_CHANNEL_6); // Se obtiene la muestra del canal ADC1_CHANNEL_6.
        }

        if (record_status) { // Si el estado de grabación sigue activo.
            audio_inference_callback(samples_to_read); // Se llama a la función de inferencia de audio.
        } else { // Si el estado de grabación no está activo.
            break; // Se sale del bucle.
        }

        vTaskDelay(pdMS_TO_TICKS(5)); // Se realiza una pausa de 5 milisegundos para permitir que el sistema realice otras tareas.
    }
    vTaskDelete(NULL); // Se elimina la tarea actual.
}

/**
 * @brief Inicializa la estructura de inferencia y configura/inicia el ADC
 *
 * @param[in] n_samples El número de muestras
 *
 * @return Verdadero si la inicialización fue exitosa, falso de lo contrario
 */
static bool microphone_inference_start(uint32_t n_samples) {
    inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));  // Asignar memoria para el buffer de muestras

    if(inference.buffer == NULL) {
        return false;  // Devolver falso si no se pudo asignar la memoria
    }

    inference.buf_count  = 0;   // Inicializar el contador de muestras en el buffer
    inference.n_samples  = n_samples;  // Establecer el número de muestras por ventana de inferencia
    inference.buf_ready  = 0;   // Inicializar el indicador de buffer listo

    record_status = true;  // Establecer el estado de grabación como activo

    // Crear tarea para capturar muestras de audio
    xTaskCreate(capture_samples, "CapturarMuestras", 1024 * 32, (void*)sample_buffer_size, 10, NULL); /*
    capture_samples: es el nombre de la función que se ejecutará cuando se active esta tarea.
"CapturarMuestras": es el nombre identificador de la tarea para su referencia.
1024 * 32: es el tamaño del stack de la tarea, en este caso, se ha asignado un tamaño de 32KB.
(void*)sample_buffer_size: es el parámetro que se pasa a la función capture_samples, en este caso, se está pasando el tamaño del buffer de muestras.
10: es la prioridad de la tarea, donde 10 es una prioridad media.
NULL: es el puntero a un objeto que se puede utilizar para recibir un identificador de la tarea creada, en este caso, no se está utilizando.
*/

    return true;  // Devolver verdadero para indicar inicialización exitosa
}

/**
 * @brief Espera nuevos datos
 *
 * @return Verdadero cuando finaliza
 */
static bool microphone_inference_record(void) {
    bool ret = true;

    while (inference.buf_ready == 0) {
        delay(5); // Ceder tiempo de CPU para otras tareas y resetear el watchdog
    }

    inference.buf_ready = 0;  // Reiniciar el indicador de buffer listo
    return ret;  // Devolver verdadero al completar la espera de nuevos datos
}

/**
 * @brief se presenta una función llamada microphone_audio_signal_get_data que se encarga de convertir muestras de señal de audio de tipo int16 a float. Esta conversión es esencial cuando se trabaja con datos de audio en diferentes formatos dentro de un proyecto de Arduino.
	@note Conceptos Clave
	Conversión de datos de audio de int16 a float.
	Uso de punteros para manipular datos en Arduino.
	Retorno de un valor para indicar el éxito de la operación.
	Estructura del Código
	La función microphone_audio_signal_get_data toma tres parámetros: offset (desplazamiento inicial en el buffer de entrada), length (longitud de datos a procesar) y out_ptr (puntero al buffer de salida donde se almacenarán los datos convertidos).
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
    numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);  // Convertir muestras de int16 a float 
 /**
*  @note ¿Qué es NumPy?
*  	NumPy es una biblioteca de Python que se utiliza para trabajar con arrays y matrices de datos numéricos. Es muy eficiente y se usa mucho en ciencia de datos, 	análisis numérico y aprendizaje automático. 
*¿Qué son int16 y float32? 
		int16: Es un tipo de dato que representa enteros de 16 bits. Puede almacenar valores enteros desde -32,768 hasta 32,767.
		float32: Es un tipo de dato que representa números de punto flotante de 32 bits. Puede almacenar números con decimales y tiene un rango mucho mayor que int16.
	¿Por qué convertir int16 a float32?
		A veces, necesitamos realizar operaciones matemáticas que requieren mayor precisión o trabajar con datos que incluyen decimales. En estos casos, convertir los datos de int16 a float32 es útil.
*/
    return 0;  // Devolver 0 para indicar éxito
}

/**
 * @brief Detiene la captura de audio y libera los buffers
 */
static void microphone_inference_end(void) {
    record_status = false;  // Establecer estado de grabación como inactivo
    ei_free(inference.buffer);  // Liberar memoria del buffer de muestras
}
