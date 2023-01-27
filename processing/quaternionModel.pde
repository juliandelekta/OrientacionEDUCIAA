import processing.net.*;

// Servidor HTTP que recibe los POST del ESP8266
Server myServer;

// Representación interna del quaternion recibido por medio de HTTP
float qx = 0.0;
float qy = 0.0;
float qz = 0.0;
float qw = 1.0;

// Matriz de rotación del canvas
PMatrix3D rotationMatrix = new PMatrix3D();
PShape car;

// Esta rutina establece los datos de una matriz de rotación "out" según las coordenadas de un quaternion (x, y, z, w)
void QuaternionToMatrix(PMatrix3D out, float x, float y, float z, float w) {
    float x2 = x + x; float y2 = y + y; float z2 = z + z;
    float xsq2 = x * x2; float ysq2 = y * y2; float zsq2 = z * z2;
    float xy2 = x * y2; float xz2 = x * z2; float yz2 = y * z2;
    float wx2 = w * x2; float wy2 = w * y2; float wz2 = w * z2;
    out.set(
      1.0 - (ysq2 + zsq2), xy2 - wz2,           xz2 + wy2,           0.0,
      xy2 + wz2,           1.0 - (xsq2 + zsq2), yz2 - wx2,           0.0,
      xz2 - wy2,           yz2 + wx2,           1.0 - (xsq2 + ysq2), 0.0,
      0.0,                 0.0,                 0.0,                 1.0
    );
}

void setup() {
    size(1024, 700, P3D); 
    
    // Inicializo el servidor en el puerto 80, puerto bien conocido para HTTP
    myServer = new Server(this, 8082);
    
    // Inicializo mi matriz de rotación con el quaternion por defecto
    QuaternionToMatrix(rotationMatrix, qx, qy, qz, qw);
    
    /*car = loadShape("Shelby.obj");
    car.scale(100);
    car.rotateX(-PI/2);
    car.translate(-100, 0, 0);*/
    
    car = loadShape("Roadster.obj");
    car.scale(40);
    car.rotateX(-PI/2);
    //car.rotateY(1.35 * PI/4);
    car.rotateZ(1.35 * PI/4);
    car.setAmbient(0xff000000);
    car.setEmissive(0xff000000);
    
    // Version1
    // car.rotateX(-PI/2);
    // car.rotateZ(1.35 * PI/4);
    
    //car.setFill(0xff336699);
    //car.setSpecular(0xff000000);
}

void draw(){
    // Pinto el fondo de gris
    background(255);
    
    lights();
    
    
    lightFalloff(0, 10, 0);
    lightSpecular(128, 128, 128);
    ambientLight(40, 40, 40);
    directionalLight(128, 128, 128, 0, 1, 0);
    pointLight(255, 127, 0,
    width * -5, cos(frameCount * 5) * height, 0);
    
    // Me traslado al centro del canvas y aplico posteriormente la matriz de rotación
    translate(width/2, height/2);
    
    // Version 1
     rotateY(PI);
     rotateX(-PI/2);
    
    //rotateY(PI);
    //rotateX(-PI/2);
    applyMatrix(rotationMatrix);
    scale(1, 1, 1);
    shape(car);
    
    
    // Get the next available client
      Client thisClient = myServer.available();
      // If the client is not null, and says something, display what it said
      if (thisClient !=null) {
        String whatClientSaid = thisClient.readString();
        if (whatClientSaid != null) {
          println(thisClient.ip() + "\t" + whatClientSaid);
          String[] q = whatClientSaid.split(" ");
          if (q.length == 4) {
              qx = float(q[0]);
              qy = float(q[1]);
              qz = float(q[2]);
              qw = float(q[3]);
              QuaternionToMatrix(rotationMatrix, qx, qy, qz, qw);
          }
        } 
      }
}
