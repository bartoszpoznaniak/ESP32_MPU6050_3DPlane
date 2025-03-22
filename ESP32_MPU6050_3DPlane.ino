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
float yprReference[3] = {0, 0, 0};
bool useReferencePosition = false;

// Flagi kalibracji postawy
bool postureCalibrationInProgress = false;
unsigned long calibrationStartTime = 0;
const unsigned long POSTURE_CALIBRATION_DURATION = 10000; // 10 sekund w milisekundach

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
      <!-- Overlay kalibracji postawy -->
      <div id="calibration-overlay" class="active">
        <div class="calibration-container">
          <h2>Kalibracja postawy</h2>
          <div id="calibration-instruction">Usiądź prosto i rozluźnij się</div>
          <div id="calibration-timer" style="display: none;">10</div>
          <div id="calibration-progress">
            <div class="progress-bar"><div class="progress-fill"></div></div>
          </div>
          <button id="start-calibration">Rozpocznij kalibrację</button>
        </div>
      </div>
      
      <h1>Monitor postawy siedzenia</h1>
      <div id="model-container"></div>
      
      <div id="posture-status">
        <div class="status-indicator good"></div>
        <div class="status-text">Poprawna postawa</div>
      </div>
      
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
  String json = "{";
  if (useReferencePosition)
  {
    // Jeśli używamy pozycji referencyjnej, odejmujemy wartości referencyjne
    json += "\"yaw\":" + String((ypr[0] * 180 / M_PI) - yprReference[0]) + ",";
    json += "\"pitch\":" + String((ypr[1] * 180 / M_PI) - yprReference[1]) + ",";
    json += "\"roll\":" + String((ypr[2] * 180 / M_PI) - yprReference[2]);
  }
  else
  {
    // Bez pozycji referencyjnej, wysyłamy wartości bezpośrednie
    json += "\"yaw\":" + String(ypr[0] * 180 / M_PI) + ",";
    json += "\"pitch\":" + String(ypr[1] * 180 / M_PI) + ",";
    json += "\"roll\":" + String(ypr[2] * 180 / M_PI);
  }
  json += "}";
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
    
    h2 {
      color: #2196F3;
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
    
    /* Style dla ekranu kalibracji postawy */
    #calibration-overlay {
      display: none;
      position: fixed;
      top: 0;
      left: 0;
      width: 100%;
      height: 100%;
      background-color: rgba(0, 0, 0, 0.8);
      z-index: 1000;
      justify-content: center;
      align-items: center;
    }
    
    #calibration-overlay.active {
      display: flex;
    }
    
    .calibration-container {
      background-color: white;
      padding: 30px;
      border-radius: 10px;
      text-align: center;
      max-width: 500px;
    }
    
    #calibration-instruction {
      font-size: 18px;
      margin: 20px 0;
    }
    
    #calibration-timer {
      font-size: 72px;
      font-weight: bold;
      color: #2196F3;
      margin: 20px 0;
    }
    
    #calibration-progress {
      margin: 20px auto;
      width: 90%;
    }
    
    .progress-bar {
      height: 20px;
      background-color: #f0f0f0;
      border-radius: 10px;
      overflow: hidden;
    }
    
    .progress-fill {
      height: 100%;
      background-color: #2196F3;
      width: 0%;
      transition: width 1s linear;
    }
    
    #start-calibration {
      padding: 15px 30px;
      font-size: 18px;
      background-color: #2196F3;
    }
    
    /* Style dla statusu postawy */
    #posture-status {
      display: flex;
      align-items: center;
      justify-content: center;
      margin: 20px auto;
      padding: 10px;
      border: 1px solid #ddd;
      border-radius: 5px;
      width: 300px;
      background-color: #f8f8f8;
    }
    
    .status-indicator {
      width: 20px;
      height: 20px;
      border-radius: 50%;
      margin-right: 10px;
    }
    
    .status-indicator.good {
      background-color: #4CAF50;
      box-shadow: 0 0 10px #4CAF50;
    }
    
    .status-indicator.warning {
      background-color: #FF9800;
      box-shadow: 0 0 10px #FF9800;
    }
    
    .status-indicator.bad {
      background-color: #f44336;
      box-shadow: 0 0 10px #f44336;
    }
    
    .status-text {
      font-weight: bold;
    }
  )rawliteral";

  server.send(200, "text/css", css);
}

// Funkcja do obsługi JavaScript
void handleJS()
{
  String js = R"rawliteral(
    // Zmienne globalne
    let scene, camera, renderer;
    let airplane;
    let lastYaw = 0, lastPitch = 0, lastRoll = 0;
    let isCalibrating = false;
    let calibrationTime = 10; // sekund
    let calibrationTimer;
    
    // Inicjalizacja sceny Three.js
    function init() {
      // Utworzenie sceny
      scene = new THREE.Scene();
      scene.background = new THREE.Color(0xf0f0f0);
      
      // Utworzenie kamery
      camera = new THREE.PerspectiveCamera(75, window.innerWidth / 400, 0.1, 1000);
      camera.position.z = 5;
      
      // Utworzenie renderera
      renderer = new THREE.WebGLRenderer({ antialias: true });
      renderer.setSize(window.innerWidth, 400);
      document.getElementById('model-container').appendChild(renderer.domElement);
      
      // Dodanie światła
      const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
      scene.add(ambientLight);
      
      const directionalLight = new THREE.DirectionalLight(0xffffff, 0.5);
      directionalLight.position.set(5, 5, 5);
      scene.add(directionalLight);
      
      // Utworzenie modelu samolotu
      createAirplane();
      
      // Obsługa zmiany rozmiaru okna
      window.addEventListener('resize', onWindowResize, false);
    }
    
    // Funkcja obsługująca zmianę rozmiaru okna
    function onWindowResize() {
      camera.aspect = window.innerWidth / 400;
      camera.updateProjectionMatrix();
      renderer.setSize(window.innerWidth, 400);
    }
    
    // Utworzenie modelu samolotu
    function createAirplane() {
      // Grupa dla całego samolotu
      airplane = new THREE.Group();
      
      // Materiały
      const bodyMaterial = new THREE.MeshPhongMaterial({ color: 0x0088cc, flatShading: true });
      const wingMaterial = new THREE.MeshPhongMaterial({ color: 0x3399ff, flatShading: true });
      
      // Korpus
      const bodyGeometry = new THREE.BoxGeometry(1, 0.4, 3);
      const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
      airplane.add(body);
      
      // Skrzydła
      const wingGeometry = new THREE.BoxGeometry(5, 0.1, 1);
      const wings = new THREE.Mesh(wingGeometry, wingMaterial);
      airplane.add(wings);
      
      // Statecznik pionowy
      const tailGeometry = new THREE.BoxGeometry(0.1, 0.8, 0.5);
      const tail = new THREE.Mesh(tailGeometry, wingMaterial);
      tail.position.y = 0.4;
      tail.position.z = -1.2;
      airplane.add(tail);
      
      // Stateczniki poziome
      const horizontalTailGeometry = new THREE.BoxGeometry(1.5, 0.1, 0.5);
      const horizontalTail = new THREE.Mesh(horizontalTailGeometry, wingMaterial);
      horizontalTail.position.z = -1.2;
      airplane.add(horizontalTail);
      
      // Dodanie samolotu do sceny
      scene.add(airplane);
    }
    
    // Animacja
    function animate() {
      requestAnimationFrame(animate);
      renderer.render(scene, camera);
    }
    // Pobieranie danych z ESP32
function fetchData() {
  fetch('/data')
    .then(response => response.json())
    .then(data => {
      // Dodajemy filtrowanie dryftu dla osi yaw
      const yawDeadzone = 0.8; // wartość progowa (w stopniach)
      if (Math.abs(data.yaw) < yawDeadzone) {
        data.yaw = 0; // zerujemy małe wartości yaw
      }
      
      // Aktualizacja wyświetlanych wartości
      document.getElementById('yaw').textContent = data.yaw.toFixed(2);
      document.getElementById('pitch').textContent = data.pitch.toFixed(2);
      document.getElementById('roll').textContent = data.roll.toFixed(2);
      
      // Łagodne przejście do nowych wartości
      lastYaw = lerp(lastYaw, data.yaw, 0.3);
      lastPitch = lerp(lastPitch, data.pitch, 0.3);
      lastRoll = lerp(lastRoll, data.roll, 0.3);
      
      // Aktualizacja rotacji samolotu
      updateAirplaneRotation(lastYaw, lastPitch, lastRoll);
      
      // Aktualizacja statusu postawy
      updatePostureStatus(data);
    })
    .catch(error => console.error('Błąd pobierania danych:', error));
}
    
    // Aktualizacja statusu postawy
    function updatePostureStatus(data) {
      const indicator = document.querySelector('.status-indicator');
      const statusText = document.querySelector('.status-text');
      
      // Obliczamy ogólne odchylenie od pozycji referencyjnej
      const deviationThreshold = 2; // Próg w stopniach
      const totalDeviation = Math.abs(data.pitch) + Math.abs(data.roll);
      
      if (totalDeviation < deviationThreshold) {
        indicator.className = 'status-indicator good';
        statusText.textContent = 'Poprawna postawa';
      } else if (totalDeviation < deviationThreshold * 2) {
        indicator.className = 'status-indicator warning';
        statusText.textContent = 'Lekkie odchylenie postawy';
      } else {
        indicator.className = 'status-indicator bad';
        statusText.textContent = 'Nieprawidłowa postawa';
      }
    }
    
    // Funkcja interpolacji liniowej
    function lerp(start, end, amt) {
      return (1 - amt) * start + amt * end;
    }
    
    // Aktualizacja rotacji samolotu
    function updateAirplaneRotation(yaw, pitch, roll) {
      if (airplane) {
        // Konwersja z stopni na radiany
        const yawRad = THREE.MathUtils.degToRad(yaw);
        const pitchRad = THREE.MathUtils.degToRad(-roll);
        const rollRad = THREE.MathUtils.degToRad(pitch);
        
        // Resetowanie rotacji
        airplane.rotation.set(0, 0, 0);
        
        // Zastosowanie nowych rotacji
        airplane.rotateZ(rollRad);
        airplane.rotateX(pitchRad);
        airplane.rotateY(yawRad);
      }
    }
    
    // Funkcja kalibracji czujnika
    function calibrateSensor() {
      document.getElementById('status-message').textContent = 'Kalibrowanie czujnika...';
      
      fetch('/calibrate')
        .then(response => response.text())
        .then(data => {
          document.getElementById('status-message').textContent = 'Kalibracja zakończona';
          setTimeout(() => {
            document.getElementById('status-message').textContent = '';
          }, 3000);
        })
        .catch(error => {
          document.getElementById('status-message').textContent = 'Błąd kalibracji';
          console.error('Błąd kalibracji:', error);
        });
    }
    
    // Funkcja resetowania czujnika
    function resetSensor() {
      document.getElementById('status-message').textContent = 'Resetowanie czujnika...';
      
      fetch('/reset')
        .then(response => response.text())
        .then(data => {
          document.getElementById('status-message').textContent = 'Reset zakończony';
          setTimeout(() => {
            document.getElementById('status-message').textContent = '';
          }, 3000);
        })
        .catch(error => {
          document.getElementById('status-message').textContent = 'Błąd resetowania';
          console.error('Błąd resetowania:', error);
        });
    }
    
    // Funkcja ustawiania pozycji referencyjnej
    function setReferencePosition() {
      document.getElementById('status-message').textContent = 'Ustawianie pozycji referencyjnej...';
      
      fetch('/setReference')
        .then(response => response.text())
        .then(data => {
          document.getElementById('status-message').textContent = 'Ustawiono pozycję referencyjną';
          setTimeout(() => {
            document.getElementById('status-message').textContent = '';
          }, 3000);
        })
        .catch(error => {
          document.getElementById('status-message').textContent = 'Błąd ustawiania pozycji';
          console.error('Błąd ustawiania pozycji:', error);
        });
    }
    
    // Funkcja rozpoczynająca kalibrację postawy
    function startPostureCalibration() {
      isCalibrating = true;
      const overlay = document.getElementById('calibration-overlay');
      const instruction = document.getElementById('calibration-instruction');
      const timerEl = document.getElementById('calibration-timer');
      const startButton = document.getElementById('start-calibration');
      const progressFill = document.querySelector('.progress-fill');
      
      startButton.style.display = 'none';
      timerEl.style.display = 'block';
      instruction.textContent = 'Pozostań nieruchomo w prawidłowej pozycji';
      
      // Ustawienie timera odliczającego 10 sekund
      let timeLeft = calibrationTime;
      timerEl.textContent = timeLeft;
      
      // Wysyłamy sygnał do ESP32, że rozpoczynamy kalibrację
      fetch('/startPostureCalibration')
        .then(response => response.text())
        .then(data => {
          console.log(data);
        });
      
      calibrationTimer = setInterval(() => {
        timeLeft--;
        timerEl.textContent = timeLeft;
        
        // Aktualizacja paska postępu
        const progress = ((calibrationTime - timeLeft) / calibrationTime) * 100;
        progressFill.style.width = `${progress}%`;
        
        if (timeLeft <= 0) {
          clearInterval(calibrationTimer);
          
          // Kalibracja zakończona
          completeCalibration();
        }
      }, 1000);
    }
    
    // Funkcja kończąca kalibrację
    function completeCalibration() {
      const overlay = document.getElementById('calibration-overlay');
      const instruction = document.getElementById('calibration-instruction');
      const timerEl = document.getElementById('calibration-timer');
      
      // Ustawienie referencyjnej pozycji
      fetch('/setPostureReference')
        .then(response => response.text())
        .then(data => {
          instruction.textContent = 'Kalibracja zakończona pomyślnie!';
          timerEl.style.display = 'none';
          
          // Po 2 sekundach ukrywamy overlay i pokazujemy główny interfejs
          setTimeout(() => {
            overlay.classList.remove('active');
            isCalibrating = false;
          }, 2000);
        });
    }
    
    // Inicjalizacja po załadowaniu strony
    document.addEventListener('DOMContentLoaded', function() {
      init();
      
      // Inicjalizacja przycisków
      document.getElementById('start-calibration').addEventListener('click', startPostureCalibration);
      
      // Rozpoczęcie pobierania danych i animacji
      animate();
      setInterval(fetchData, 250); // Pobieranie danych co 250ms zamiast 100ms
    });
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

// Funkcja rozpoczynająca kalibrację postawy
void handleStartPostureCalibration()
{
  Serial.println("Rozpoczęcie kalibracji postawy...");
  postureCalibrationInProgress = true;
  calibrationStartTime = millis();
  server.send(200, "text/plain", "Rozpoczęto proces kalibracji postawy");
}

// Funkcja ustawiająca referencyjną pozycję postawy
void handleSetPostureReference()
{
  // Ustawienie bieżącej wartości jako referencyjnej
  yprReference[0] = ypr[0] * 180 / M_PI;
  yprReference[1] = ypr[1] * 180 / M_PI;
  yprReference[2] = ypr[2] * 180 / M_PI;
  useReferencePosition = true;

  Serial.println("Ustawiono referencyjną pozycję postawy:");
  Serial.print("Yaw: ");
  Serial.print(yprReference[0]);
  Serial.print(", Pitch: ");
  Serial.print(yprReference[1]);
  Serial.print(", Roll: ");
  Serial.println(yprReference[2]);

  server.send(200, "text/plain", "Ustawiono referencyjną pozycję postawy");
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
  server.on("/startPostureCalibration", HTTP_GET, handleStartPostureCalibration);
  server.on("/setPostureReference", HTTP_GET, handleSetPostureReference);

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
      else if (postureCalibrationInProgress)
      {
        // Sprawdź czy upłynął czas kalibracji postawy
        if (currentMillis - calibrationStartTime >= POSTURE_CALIBRATION_DURATION)
        {
          postureCalibrationInProgress = false;
          Serial.println("Kalibracja postawy zakończona automatycznie");
        }

        // W czasie kalibracji postawy tylko odczytujemy dane, ale nie zmieniamy żadnych offsetów
        // Pobranie wartości Eulera (yaw, pitch, roll)
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
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
} // kuku 15.1