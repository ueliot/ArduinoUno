 #include <Arduino_FreeRTOS.h>

#define LED1  3  //Set GPIO 25 as  CAP0
#define LED2  4

void Task1(void *pvParameters)
{
  (void) pvParameters;

  pinMode(LED1, OUTPUT);

  while(true)
  {
    digitalWrite(LED1, HIGH);
    vTaskDelay( 100 / portTICK_PERIOD_MS );
    digitalWrite(LED1, LOW);    
    vTaskDelay( 500 / portTICK_PERIOD_MS );
  }
}

void Task2(void *pvParameters)
{
  (void) pvParameters;

  pinMode(LED2, OUTPUT);

  while(true)
  {
    // digitalWrite(LED2, HIGH);
    // vTaskDelay( 500 / portTICK_PERIOD_MS );
    // digitalWrite(LED2, LOW);    
    // vTaskDelay( 500 / portTICK_PERIOD_MS );
  }
}

void setup()
{
  //Serial.begin(9600);

  xTaskCreate(
    Task1
    ,  "Blink"    // Nombre de la tarea
    ,  128      // Tamaño de la pila de la tarea
    ,  NULL
    ,  2        // Prioridad, siendo 0 la de menor prioridad, y 3 la de mayor
    ,  NULL );
    xTaskCreate(Task2,  "Blink" ,  128 ,  NULL,  2 , NULL );
}



void loop()
{
  // Serial.println(millis());
  // delay(1000);
}