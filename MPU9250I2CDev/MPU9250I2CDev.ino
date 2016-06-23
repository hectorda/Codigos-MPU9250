// Arduino library and code are available to download - link below the video

//Connection pins provided in the diagram at the beginning of video

// library provided and code based on: https://github.com/Seeed-Studio/IMU_10DOF
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU9250.h"

// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 MPU;

enum aScale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};

enum gScale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

int Ascale=AFS_2G;

//Configuracion Giroscopio // Set the scale below either 250, 500 ,1000  o 2000
int Gscale=GFS_250DPS;
//Frecuencia de Muestreo
int rate=39; //39 para 200 Hz
//Filtro Pasa-Bajo Digital
int dlpf=0;
int cont=0;

double A_R[4]={16384,8192,4096,2048};
double G_R[4]={131,65.5,32.8,16.4};

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Serial.begin(115200);
  //Serial.println("Initializing I2C devices...");
  MPU.initialize();
  //Serial.println("Testing device connections...");
  //Serial.println(MPU.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");
  //delay(1000);
}

void loop()
{
    //Escuchando para cambiar :)  
     if (Serial.available()){
          char key = Serial.read();
          char value= Serial.parseInt();
          cambiarConfiguraciones(key,value);
          cont++;
    }
    if (cont==4){    
      if(MPU.getIntStatus()) //Se ve si hay interrupcion
        obtenerDatos();
    }
}

void obtenerDatos(){
    MPU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);    
    Serial.print(ax/A_R[Ascale]); Serial.print(" ");
    Serial.print(ay/A_R[Ascale]); Serial.print(" ");
    Serial.print(az/A_R[Ascale]); Serial.print(" ");
    Serial.print(gx/G_R[Gscale]); Serial.print(" ");
    Serial.print(gy/G_R[Gscale]); Serial.print(" ");
    Serial.println(gz/G_R[Gscale]);
}


void cambiarConfiguraciones(char key,int value)
{
  if (key == 'g'){
    switch (value) {
      case 0:
        Gscale=GFS_250DPS;
        MPU.setFullScaleGyroRange(Gscale);
        //Serial.print("RangoGiroscopio: 250dps");
        break;
      case 1:
        Gscale=GFS_500DPS;
        MPU.setFullScaleGyroRange(Gscale);
        //Serial.print("RangoGiroscopio: 500dps");
        break;
      case 2:
        Gscale=GFS_1000DPS;
        MPU.setFullScaleGyroRange(Gscale);
        //Serial.print("RangoGiroscopio: 1000dps");
        break;
      case 3:
        Gscale=GFS_2000DPS;
        MPU.setFullScaleGyroRange(Gscale);
        //Serial.print("RangoGiroscopio: 2000dps");
        break;
      
      default: 
        // if nothing else matches, do the default
        // default is optional
      break;
    }
  }
  if (key == 'a'){
    switch (value) {
      case 0:
        Ascale=AFS_2G;
        MPU.setFullScaleAccelRange(Ascale);
        //Serial.print("RangoAcelerometro: 2G");
        break;
      case 1:
        Ascale=AFS_4G;
        MPU.setFullScaleAccelRange(Ascale);
        //Serial.print("RangoAcelerometro: 4G");
        break;
      case 2:
        Ascale=AFS_8G;
        MPU.setFullScaleAccelRange(Ascale);
        //Serial.print("RangoAcelerometro: 8G");
        break;
      case 3:
        Ascale=AFS_16G;
        MPU.setFullScaleAccelRange(Ascale);
        //Serial.print("RangoAcelerometro: 16G");
        break;
      
      default: 
        // if nothing else matches, do the default
        // default is optional
      break;
    }
  }
  if (key == 's'){
    rate=value;
    MPU.setRate(rate);
  }
  if (key == 'l'){
    dlpf=value;
    MPU.setDLPFMode(dlpf);
  }
  return;
}


