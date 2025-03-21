// ESP32 + MPU6050 + wizualizacja 3D samolotu
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

// Konfiguracja WiFi - zastąp swoimi danymi
const char *ssid = "ASUS";                 // Zmień na nazwę swojej sieci WiFi
const char *password = "Bananysafajne123"; // Zmień na hasło swojej sieci WiFi

// Tworzenie instancji serwera WWW na porcie 80
WebServer server(80);

// Instancja MPU6050
MPU6050 mpu;

// MPU kontrolne/statusowe zmienne
bool dmpReady = false;  // zmienna pomocnicza dla sprawdzenia poprawnej inicjalizacji DMP
uint8_t mpuIntStatus;   // status przerwania z MPU
uint8_t devStatus;      // status urządzenia (0 = sukces, !0 = błąd)
uint16_t packetSize;    // oczekiwany rozmiar pakietu DMP (domyślnie 42 bajty)
uint16_t fifoCount;     // licznik bajtów obecnie w buforze FIFO
uint8_t fifoBuffer[64]; // bufor FIFO

// Zmienne orientacji/ruchu
Quaternion q;        // [w, x, y, z]         kwaternion
VectorFloat gravity; // [x, y, z]            wektor grawitacji
float ypr[3];        // [yaw, pitch, roll]   wartości w radianach

// Buforowanie danych i czas
unsigned long lastDataUpdate = 0;
const unsigned long DATA_UPDATE_INTERVAL = 20; // Aktualizacja danych co 20ms

// Zmienne referencyjne (dla pozycji zerowej)
float yprReference[3] = {0, 0, 0}; // Wartości referencyjne dla yaw, pitch, roll
bool useReferencePosition = false; // Czy używać pozycji referencyjnej

// Zmienne kalibracyjne
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t gxOffset = 0, gyOffset = 0, gzOffset = 0;
int16_t axOffset = 0, ayOffset = 0, azOffset = 0;
bool calibrationInProgress = false;
int calibrationSamples = 0;
const int CALIBRATION_SAMPLES = 100;
int32_t gxSum = 0, gySum = 0, gzSum = 0;
int32_t axSum = 0, aySum = 0, azSum = 0;

// Definicja pinów
const int SDA_PIN = 21;
const int SCL_PIN = 22;

// Obsługa głównej strony
void handleRoot()
{
  String html = R"rawliteral(
    <!DOCTYPE html>
    <html lang="pl">
    <head>
      <meta charset="UTF-8">
      <meta name="viewport" content="width=device-width, initial-scale=1.0">
      <title>Wizualizacja 3D MPU6050</title>
      <link rel="stylesheet" href="styles.css">
      <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    </head>
    <body>
      <h1>Wizualizacja MPU6050</h1>
      <div id="model-container"></div>
      <div id="data-display">
        <p>Yaw: <span id="yaw">0</span>°</p>
        <p>Pitch: <span id="pitch">0</span>°</p>
        <p>Roll: <span id="roll">0</span>°</p>
      </div>
      <div id="control-panel">
        <button id="set-reference-btn" onclick="setReferencePosition()" class="primary-btn">Ustaw bieżącą pozycję jako zerową</button>
        <button id="calibrate-btn" onclick="calibrateSensor()">Kalibruj czujnik</button>
        <button id="reset-btn" onclick="resetSensor()">Reset czujnika</button>
        <div id="status-message"></div>
      </div>
      <script src="scripts.js"></script>
    </body>
    </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

// Funkcja do wysyłania danych jako JSON
void handleData()
{
  float yawValue, pitchValue, rollValue;

  if (useReferencePosition)
  {
    // Odejmij wartości referencyjne, aby ustalić bieżącą pozycję względem pozycji zerowej
    yawValue = ypr[0] * 180 / M_PI - yprReference[0];
    pitchValue = ypr[1] * 180 / M_PI - yprReference[1];
    rollValue = ypr[2] * 180 / M_PI - yprReference[2];
  }
  else
  {
    // Użyj bezwzględnych wartości
    yawValue = ypr[0] * 180 / M_PI;
    pitchValue = ypr[1] * 180 / M_PI;
    rollValue = ypr[2] * 180 / M_PI;
  }

  String json = "{\"yaw\":" + String(yawValue) +
                ",\"pitch\":" + String(pitchValue) +
                ",\"roll\":" + String(rollValue) + "}";
  server.send(200, "application/json", json);
}

// Funkcja do obsługi CSS
void handleCSS()
{
  String css = R"rawliteral(
    body {
      font-family: Arial, sans-serif;
      margin: 0;
      padding: 20px;
      text-align: center;
    }
    
    h1 {
      color: #333;
    }
    
    #model-container {
      margin: 20px auto;
      width: 100%;
      height: 400px;
      background-color: #f0f0f0;
    }
    
    #data-display {
      margin: 20px auto;
      padding: 10px;
      border: 1px solid #ddd;
      border-radius: 5px;
      width: 300px;
      background-color: #f8f8f8;
    }
    
    #data-display p {
      margin: 5px;
      font-size: 18px;
    }
    
    #control-panel {
      margin: 20px auto;
      padding: 10px;
      border: 1px solid #ddd;
      border-radius: 5px;
      width: 300px;
      background-color: #f0f8ff;
    }
    
    button {
      padding: 10px 15px;
      margin: 5px;
      background-color: #4CAF50;
      color: white;
      border: none;
      border-radius: 4px;
      cursor: pointer;
      font-size: 16px;
    }
    
    button:hover {
      background-color: #45a049;
    }
    
    .primary-btn {
      background-color: #2196F3;
      width: 100%;
      margin-bottom: 10px;
    }
    
    .primary-btn:hover {
      background-color: #0b7dda;
    }
    
    #reset-btn {
      background-color: #f44336;
    }
    
    #reset-btn:hover {
      background-color: #d32f2f;
    }
    
    #status-message {
      margin-top: 10px;
      padding: 5px;
      color: #666;
      font-style: italic;
    }
  )rawliteral";

  server.send(200, "text/css", css);
}

// Funkcja do obsługi JavaScript
void handleJS()
{
  String js = R"rawliteral(
    // Inicjalizacja Three.js
    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    const renderer = new THREE.WebGLRenderer();
    
    renderer.setSize(window.innerWidth * 0.8, 400);
    document.getElementById('model-container').appendChild(renderer.domElement);
    
    // Dodanie oświetlenia
    const ambientLight = new THREE.AmbientLight(0x404040);
    scene.add(ambientLight);
    
    const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
    directionalLight.position.set(0, 1, 0);
    scene.add(directionalLight);
    
    // Tworzenie prostego modelu samolotu
    function createPlane() {
      const group = new THREE.Group();
      
      // Korpus
      const bodyGeometry = new THREE.BoxGeometry(2, 0.5, 5);
      const bodyMaterial = new THREE.MeshPhongMaterial({ color: 0x3333ff });
      const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
      group.add(body);
      
      // Skrzydła
      const wingGeometry = new THREE.BoxGeometry(7, 0.1, 1);
      const wingMaterial = new THREE.MeshPhongMaterial({ color: 0x3333aa });
      const wing = new THREE.Mesh(wingGeometry, wingMaterial);
      wing.position.y = 0;
      group.add(wing);
      
      // Ogon - część pionowa
      const tailFinGeometry = new THREE.BoxGeometry(0.1, 1, 0.8);
      const tailFinMaterial = new THREE.MeshPhongMaterial({ color: 0x3333aa });
      const tailFin = new THREE.Mesh(tailFinGeometry, tailFinMaterial);
      tailFin.position.z = -2;
      tailFin.position.y = 0.5;
      group.add(tailFin);
      
      // Ogon - część pozioma
      const tailWingGeometry = new THREE.BoxGeometry(2, 0.1, 0.8);
      const tailWingMaterial = new THREE.MeshPhongMaterial({ color: 0x3333aa });
      const tailWing = new THREE.Mesh(tailWingGeometry, tailWingMaterial);
      tailWing.position.z = -2;
      tailWing.position.y = 0.2;
      group.add(tailWing);
      
      return group;
    }
    
    const airplane = createPlane();
    scene.add(airplane);
    
    // Ustawienie kamery
    camera.position.z = 10;
    
    // Funkcja do aktualizacji wartości na stronie
    function updateDataDisplay(data) {
      document.getElementById('yaw').textContent = data.yaw.toFixed(2);
      document.getElementById('pitch').textContent = data.pitch.toFixed(2);
      document.getElementById('roll').textContent = data.roll.toFixed(2);
    }
    
    // Funkcja do pobierania danych z ESP32
    function fetchData() {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          // Aktualizacja wyświetlanych danych
          updateDataDisplay(data);
          
          // Konwersja stopni na radiany
          const yaw = data.yaw * Math.PI / 180;
          const pitch = data.pitch * Math.PI / 180;
          const roll = data.roll * Math.PI / 180;
          
          // Resetowanie rotacji
          airplane.rotation.set(0, 0, 0);
          
          // Zastosowanie rotacji w odpowiedniej kolejności
          // Uwaga: kolejność rotacji jest ważna!
          airplane.rotateZ(roll);    // Roll - wokół osi Z
          airplane.rotateX(pitch);   // Pitch - wokół osi X
          airplane.rotateY(yaw);     // Yaw - wokół osi Y
        })
        .catch(error => console.error('Błąd pobierania danych:', error));
    }
    
    // Funkcja do ustawiania bieżącej pozycji jako pozycji zerowej
    function setReferencePosition() {
      const statusMessage = document.getElementById('status-message');
      statusMessage.textContent = 'Ustawianie pozycji referencyjnej...';
      
      fetch('/setReference')
        .then(response => response.text())
        .then(data => {
          statusMessage.textContent = data;
          setTimeout(() => {
            statusMessage.textContent = '';
          }, 5000);
        })
        .catch(error => {
          console.error('Błąd ustawiania pozycji referencyjnej:', error);
          statusMessage.textContent = 'Błąd ustawiania pozycji referencyjnej!';
        });
    }
    
    // Funkcja do kalibracji czujnika
    function calibrateSensor() {
      const statusMessage = document.getElementById('status-message');
      statusMessage.textContent = 'Kalibracja w toku... Utrzymaj czujnik nieruchomo.';
      
      fetch('/calibrate')
        .then(response => response.text())
        .then(data => {
          statusMessage.textContent = data;
          setTimeout(() => {
            statusMessage.textContent = '';
          }, 5000);
        })
        .catch(error => {
          console.error('Błąd kalibracji:', error);
          statusMessage.textContent = 'Błąd kalibracji!';
        });
    }
    
    // Funkcja do resetowania czujnika
    function resetSensor() {
      const statusMessage = document.getElementById('status-message');
      statusMessage.textContent = 'Resetowanie czujnika...';
      
      fetch('/reset')
        .then(response => response.text())
        .then(data => {
          statusMessage.textContent = data;
          setTimeout(() => {
            statusMessage.textContent = '';
          }, 5000);
        })
        .catch(error => {
          console.error('Błąd resetowania:', error);
          statusMessage.textContent = 'Błąd resetowania!';
        });
    }
    
    // Funkcja animacji
    function animate() {
      requestAnimationFrame(animate);
      renderer.render(scene, camera);
    }
    
    // Obsługa zmiany rozmiaru okna
    window.addEventListener('resize', () => {
      camera.aspect = window.innerWidth / window.innerHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(window.innerWidth * 0.8, 400);
    });
    
    // Rozpoczęcie pobierania danych i animacji
    animate();
    setInterval(fetchData, 250); // Pobieranie danych co 250ms zamiast 100ms
  )rawliteral";

  server.send(200, "application/javascript", js);
}

// Funkcja do obsługi kalibracji MPU6050
void handleCalibrate()
{
  if (calibrationInProgress)
  {
    server.send(200, "text/plain", "Kalibracja już w toku!");
    return;
  }

  // Reset zmiennych kalibracyjnych
  gxSum = 0;
  gySum = 0;
  gzSum = 0;
  axSum = 0;
  aySum = 0;
  azSum = 0;
  calibrationSamples = 0;
  calibrationInProgress = true;

  server.send(200, "text/plain", "Rozpoczęto kalibrację. Utrzymaj czujnik nieruchomo przez kilka sekund.");

  Serial.println("Rozpoczęcie kalibracji. Utrzymaj czujnik nieruchomo!");
}

// Funkcja do obsługi resetu MPU6050
void handleReset()
{
  Serial.println("Resetowanie MPU6050...");

  // Reset MPU6050
  mpu.reset();
  delay(50);

  // Ponowna inicjalizacja DMP
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // Zastosowanie ostatnich offsetów kalibracyjnych
  mpu.setXGyroOffset(gxOffset);
  mpu.setYGyroOffset(gyOffset);
  mpu.setZGyroOffset(gzOffset);
  mpu.setXAccelOffset(axOffset);
  mpu.setYAccelOffset(ayOffset);
  mpu.setZAccelOffset(azOffset);

  if (devStatus == 0)
  {
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    server.send(200, "text/plain", "MPU6050 zresetowano pomyślnie!");
    Serial.println("MPU6050 zresetowano pomyślnie");
  }
  else
  {
    server.send(200, "text/plain", "Błąd resetu MPU6050!");
    Serial.print("Błąd inicjalizacji DMP po resecie (kod ");
    Serial.print(devStatus);
    Serial.println(")");
  }
}

// Funkcja do ustawiania bieżącej pozycji jako pozycji referencyjnej (zerowej)
void handleSetReference()
{
  // Zapisz bieżące wartości jako referencyjne
  yprReference[0] = ypr[0] * 180 / M_PI; // Yaw
  yprReference[1] = ypr[1] * 180 / M_PI; // Pitch
  yprReference[2] = ypr[2] * 180 / M_PI; // Roll

  useReferencePosition = true;

  server.send(200, "text/plain", "Ustawiono bieżącą pozycję jako referencyjną (zerową).");

  Serial.println("Ustawiono pozycję referencyjną:");
  Serial.print("Yaw: ");
  Serial.print(yprReference[0]);
  Serial.print(", Pitch: ");
  Serial.print(yprReference[1]);
  Serial.print(", Roll: ");
  Serial.println(yprReference[2]);
}

void setup()
{
  Serial.begin(115200);

  // Inicjalizacja SPIFFS (do przechowywania plików WWW)
  if (!SPIFFS.begin(true))
  {
    Serial.println("Błąd przy montowaniu SPIFFS");
    return;
  }

  // Inicjalizacja I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // 400kHz I2C clock

  // Inicjalizacja MPU6050
  Serial.println("Inicjalizacja MPU6050...");
  mpu.initialize();

  // Sprawdzenie połączenia
  Serial.println("Testowanie połączenia z urządzeniem...");
  Serial.println(mpu.testConnection() ? "MPU6050 połączenie udane" : "MPU6050 połączenie nieudane");

  // Inicjalizacja DMP
  Serial.println("Inicjalizacja DMP...");
  devStatus = mpu.dmpInitialize();

  // Ustawienie wartości kalibracyjnych - dostosuj te wartości do swojego urządzenia
  gxOffset = 220;
  gyOffset = 76;
  gzOffset = -85;
  axOffset = 0;
  ayOffset = 0;
  azOffset = 1788;

  mpu.setXGyroOffset(gxOffset);
  mpu.setYGyroOffset(gyOffset);
  mpu.setZGyroOffset(gzOffset);
  mpu.setZAccelOffset(azOffset);

  // Sprawdzenie czy wszystko OK
  if (devStatus == 0)
  {
    // Włączenie DMP
    Serial.println("Włączanie DMP...");
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;

    // Pobranie oczekiwanego rozmiaru pakietu DMP
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // Błąd inicjalizacji - 1 = błąd ładowania pamięci, 2 = błąd konfiguracji DMP
    Serial.print("Błąd inicjalizacji DMP (kod ");
    Serial.print(devStatus);
    Serial.println(")");
  }

  // Konfiguracja WiFi
  WiFi.begin(ssid, password);
  Serial.println("Łączenie z WiFi...");

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi połączone.");
  Serial.print("Adres IP: ");
  Serial.println(WiFi.localIP());

  // Konfiguracja ścieżek serwera WWW
  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/styles.css", HTTP_GET, handleCSS);
  server.on("/scripts.js", HTTP_GET, handleJS);
  server.on("/calibrate", HTTP_GET, handleCalibrate);
  server.on("/reset", HTTP_GET, handleReset);
  server.on("/setReference", HTTP_GET, handleSetReference);

  // Uruchomienie serwera
  server.begin();
  Serial.println("Serwer HTTP uruchomiony");
}

void loop()
{
  // Jeśli wystąpił błąd inicjalizacji, nie kontynuuj
  if (!dmpReady)
    return;

  // Obsługa zapytań do serwera
  server.handleClient();

  // Aktualizuj dane tylko co określony czas
  unsigned long currentMillis = millis();
  if (currentMillis - lastDataUpdate >= DATA_UPDATE_INTERVAL)
  {
    lastDataUpdate = currentMillis;

    // Pobierz najnowsze dane z MPU6050
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    // Sprawdzenie przepełnienia FIFO
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
      mpu.resetFIFO();
      Serial.println("Przepełnienie FIFO!");
    }
    else if (mpuIntStatus & 0x02)
    {
      // Czekanie na dostępność danych
      while (fifoCount < packetSize)
        fifoCount = mpu.getFIFOCount();

      // Odczyt pakietu z FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;

      // Jeśli kalibracja jest w toku, zbierz dane do kalibracji
      if (calibrationInProgress)
      {
        // Pobierz surowe dane z akcelerometru i żyroskopu
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // Sumuj odczyty
        gxSum += gx;
        gySum += gy;
        gzSum += gz;
        axSum += ax;
        aySum += ay;
        azSum += az;

        calibrationSamples++;

        // Po zebraniu wystarczającej liczby próbek, oblicz offsety
        if (calibrationSamples >= CALIBRATION_SAMPLES)
        {
          // Oblicz średnie wartości
          gxOffset = gxSum / CALIBRATION_SAMPLES;
          gyOffset = gySum / CALIBRATION_SAMPLES;
          gzOffset = gzSum / CALIBRATION_SAMPLES;

          // Dla akcelerometru, offset to różnica między wartością a 0 dla X i Y oraz 16384 (1g) dla Z
          axOffset = axSum / CALIBRATION_SAMPLES;
          ayOffset = aySum / CALIBRATION_SAMPLES;
          azOffset = (azSum / CALIBRATION_SAMPLES) - 16384; // Około 1g dla osi Z

          // Zastosuj nowe offsety
          mpu.setXGyroOffset(gxOffset);
          mpu.setYGyroOffset(gyOffset);
          mpu.setZGyroOffset(gzOffset);
          mpu.setXAccelOffset(axOffset);
          mpu.setYAccelOffset(ayOffset);
          mpu.setZAccelOffset(azOffset);

          Serial.println("Kalibracja zakończona!");
          Serial.print("Nowe offsety: gX=");
          Serial.print(gxOffset);
          Serial.print(" gY=");
          Serial.print(gyOffset);
          Serial.print(" gZ=");
          Serial.print(gzOffset);
          Serial.print(" aX=");
          Serial.print(axOffset);
          Serial.print(" aY=");
          Serial.print(ayOffset);
          Serial.print(" aZ=");
          Serial.println(azOffset);

          calibrationInProgress = false;

          // Reset FIFO po kalibracji
          mpu.resetFIFO();
        }
      }
      else
      {
        // Pobranie wartości Eulera (yaw, pitch, roll)
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Wyświetlenie wartości Eulera dla debugowania - tylko co ok. 500ms
        static unsigned long lastDebugOutput = 0;
        if (currentMillis - lastDebugOutput > 500)
        {
          lastDebugOutput = currentMillis;
          Serial.print("ypr\t");
          Serial.print(ypr[0] * 180 / M_PI);
          Serial.print("\t");
          Serial.print(ypr[1] * 180 / M_PI);
          Serial.print("\t");
          Serial.println(ypr[2] * 180 / M_PI);
        }
      }
    }
  }
} // kuku 6