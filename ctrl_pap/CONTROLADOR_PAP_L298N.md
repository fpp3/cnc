# Controlador de Motor Paso a Paso Bipolar (L298N + 74LS175 + CD4052)

Documentación completa de diseño, conexiones pin a pin, lista de materiales y guía de simulación para el módulo de control y potencia de motor paso a paso bipolar (NEMA 17 / NEMA 23).

---

## 1. Arquitectura y Principio de Funcionamiento

El controlador implementa la secuencia de **paso completo con dos fases activadas simultáneamente** (*two-phase-on full stepping*), entregando el máximo torque de retención y una rotación en cuadratura espacial de 90°.

### Por qué se utiliza la combinación 74LS175 + CD4052
A diferencia de los registros de desplazamiento de 4 bits (como el CD40194, que tienen 12 estados ilegales y pueden trabarse en `0000` si no se precargan), este circuito opera como un **contador en anillo de 2 bits ($Q_A, Q_B$)**.

* **Cero estados ilegales:** Al tener solo 2 bits, el circuito posee exactamente $2^2 = 4$ estados posibles ($00, 10, 11, 01$). Todos los estados son válidos.
* **Arranque garantizado sin trucos:** Al energizar la fuente, caiga donde caiga el silicio, ya se encuentra en un paso válido. No requiere pulsadores de inicio, diodos de precarga ni redes RC.
* **Salidas directas al puente:** El integrado 74LS175 entrega internamente las salidas directas e invertidas ($Q$ y $\bar{Q}$), enviando las 4 señales directamente a los pines de entrada del L298N sin compuertas inversoras externas.
* **Control de sentido limpio:** El multiplexor CD4052 conmuta las entradas de datos según el estado del pin `DIR` para invertir el sentido de giro al instante.

### Tabla de Estados de Cuadratura (Salidas hacia L298N)

| Paso | $Q_A$ (IN1) | $\bar{Q}_A$ (IN2) | $Q_B$ (IN3) | $\bar{Q}_B$ (IN4) | Fase A | Fase B | Sentido Horario ($\text{DIR}=1$) |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **0** | 0 | 1 | 0 | 1 | $-I$ | $-I$ | Paso inicial |
| **1** | 1 | 0 | 0 | 1 | $+I$ | $-I$ | Avance $\downarrow$ |
| **2** | 1 | 0 | 1 | 0 | $+I$ | $+I$ | Avance $\downarrow$ |
| **3** | 0 | 1 | 1 | 0 | $-I$ | $+I$ | Avance $\downarrow$ |

Para $\text{DIR}=0$ (Antihorario), la secuencia conmuta en sentido inverso: $0 \to 3 \to 2 \to 1 \to 0$.

---

## 2. Conexión Pin a Pin Completa

### 2.1. CD4052 (DIP-16) - Multiplexor de Dirección
El integrado actúa como dos llaves selectoras de 2 canales controladas por el pin `DIR`. El pin de dirección B se conecta a masa para fijar la mitad inferior de los canales.

| Pin CD4052 | Nombre | Función / Conexión |
| :---: | :--- | :--- |
| **1** | $Y_0$ | Conectado a $Q_A$ (Pin 2 del 74LS175) |
| **2** | $Y_2$ | A Masa (GND) (canal no utilizado) |
| **3** | **Salida Y** | Conectado a $D_B$ (Pin 5 del 74LS175) |
| **4** | $Y_3$ | A Masa (GND) (canal no utilizado) |
| **5** | $Y_1$ | Conectado a $\bar{Q}_A$ (Pin 3 del 74LS175) |
| **6** | $\overline{\text{INH}}$ | A Masa (GND) (habilitación permanente del chip) |
| **7** | $V_{EE}$ | A Masa (GND) |
| **8** | $V_{SS}$ | A Masa (GND) |
| **9** | Dirección B | A Masa (GND) |
| **10** | **Dirección A** | **A la línea de entrada `DIR`** |
| **11** | $X_3$ | A Masa (GND) (canal no utilizado) |
| **12** | $X_0$ | Conectado a $\bar{Q}_B$ (Pin 6 del 74LS175) |
| **13** | **Salida X** | Conectado a $D_A$ (Pin 4 del 74LS175) |
| **14** | $X_1$ | Conectado a $Q_B$ (Pin 7 del 74LS175) |
| **15** | $X_2$ | A Masa (GND) (canal no utilizado) |
| **16** | $V_{DD}$ | **A +5V** |

### 2.2. 74LS175 (DIP-16) - Biestables D de Memoria
Se utilizan dos de los cuatro biestables disponibles.

| Pin 74LS175 | Nombre | Función / Conexión |
| :---: | :--- | :--- |
| **1** | $\overline{\text{MR}}$ | A +5V (Master Reset deshabilitado) |
| **2** | **$Q_0$** | **Al L298N Pin 5 (IN1)** y retorno a Pin 1 del CD4052 |
| **3** | **$\bar{Q}_0$** | **Al L298N Pin 7 (IN2)** y retorno a Pin 5 del CD4052 |
| **4** | **$D_0$** | Desde Salida X del CD4052 (Pin 13) |
| **5** | **$D_1$** | Desde Salida Y del CD4052 (Pin 3) |
| **6** | **$\bar{Q}_1$** | **Al L298N Pin 12 (IN4)** y retorno a Pin 12 del CD4052 |
| **7** | **$Q_1$** | **Al L298N Pin 10 (IN3)** y retorno a Pin 14 del CD4052 |
| **8** | GND | A Masa (GND) |
| **9** | **CP (Clock)** | **A la línea de entrada `STEP`** |
| **10 a 15** | FF2 y FF3 | Entradas a masa (biestables no utilizados) |
| **16** | $V_{CC}$ | **A +5V** |

### 2.3. L298N (Multiwatt-15) - Doble Puente H de Potencia

| Pin L298N | Nombre | Función / Conexión |
| :---: | :--- | :--- |
| **1** | SENSE A | Retorno de potencia Fase A (a masa por shunt de $0.5\,\Omega$) |
| **2** | OUT1 | Salida terminal 1 de Bobina Fase A |
| **3** | OUT2 | Salida terminal 2 de Bobina Fase A |
| **4** | $V_S$ | Alimentación motriz (+12V a +24V desde bornera `J_PWR`) |
| **5** | **IN1** | Desde Pin 2 del 74LS175 ($Q_0$) |
| **6** | **ENA** | Habilitación Puente A (puenteado con Pin 11 y a línea `ENABLE`) |
| **7** | **IN2** | Desde Pin 3 del 74LS175 ($\bar{Q}_0$) |
| **8** | GND | Masa de referencia y potencia común |
| **9** | $V_{SS}$ | Alimentación lógica (+5V generados por el LM7805) |
| **10** | **IN3** | Desde Pin 7 del 74LS175 ($Q_1$) |
| **11** | **ENB** | Habilitación Puente B (puenteado con Pin 6) |
| **12** | **IN4** | Desde Pin 6 del 74LS175 ($\bar{Q}_1$) |
| **13** | OUT3 | Salida terminal 1 de Bobina Fase B |
| **14** | OUT4 | Salida terminal 2 de Bobina Fase B |
| **15** | SENSE B | Retorno de potencia Fase B (a masa por shunt de $0.5\,\Omega$) |

---

## 3. Conectores de la Placa (100% Borneras a Tornillo KF301)

Para garantizar inmunidad frente a vibraciones mecánicas y evitar falsos contactos de cables sueltos, todas las conexiones exteriores se realizan mediante borneras a tornillo paso 5.0 mm / 5.08 mm (KF301):

```text
  [ J_PWR ]         [ J_MOT ]              [ J_CTRL ]            [ J_SNS ]
 +---------+   +-----------------+   +---------------------+   +---------------+
 | +V  GND |   | OUT1 OUT2 OUT3 OUT4 |   | GND  EN   DIR  STP  |   | SNS_A GND SNS_B |
 +---------+   +-----------------+   +---------------------+   +---------------+
   2 Vías            4 Vías                  4 Vías                 3 Vías
```

1. **`J_PWR` (Bornera 2 vías):**
   - Pin 1: `+V_MOT` (+12V a +24V de alimentación de potencia del motor).
   - Pin 2: `GND` (Masa de la fuente de potencia).
2. **`J_MOT` (Bornera 4 vías):**
   - Pines 1 y 2: `OUT1` y `OUT2` (Bobina Fase A).
   - Pines 3 y 4: `OUT3` y `OUT4` (Bobina Fase B).
3. **`J_CTRL` (Bornera 4 vías):**
   - Pin 1: `GND` (Masa común obligatoria con el controlador externo / CNC).
   - Pin 2: `ENABLE` (Habilitación general del puente H).
   - Pin 3: `DIR` (Sentido de giro: 5V Horario, 0V Antihorario).
   - Pin 4: `STEP` (Pulsos de paso).
   *(Nota: no requiere entrada de 5V porque la placa genera sus propios 5V con el regulador LM7805 integrado).*
4. **`J_SNS` (Bornera 3 vías):**
   - Pin 1: `SNS_A` (Tensión proporcional a la corriente en Fase A).
   - Pin 2: `GND` (Masa de referencia).
   - Pin 3: `SNS_B` (Tensión proporcional a la corriente en Fase B).
   *(Derivación analógica para adquisición y detección de tope mecánico por corriente).*

---

## 4. Circuito de Habilitación (ENABLE)

La línea `ENABLE` controla los pines 6 y 11 del L298N:
* **Con `ENABLE = 5V`:** Los puentes H conducen normalmente, reteniendo el motor o dando pasos.
* **Con `ENABLE = 0V`:** Todos los transistores se abren (alta impedancia). La corriente cae a 0 mA, el motor entra en "rueda libre" para mover los ejes a mano, y se anula la disipación térmica en reposo.

### Circuito asociado:
1. **Pull-up de seguridad:** Resistencia de $10\text{ k}\Omega$ conectada a +5V en la línea `ENABLE` para que el driver arranque activo si no se conecta el cable de control.
2. **Selector manual (Jumper 1x03 en placa):**
   - Pin 1: +5V fijo (`ON` forzado).
   - Pin 2: Al puente ENA/ENB del L298N.
   - Pin 3: A la bornera `J_CTRL` (Control externo).
3. **LED Testigo:** LED verde con resistencia de $1\text{ k}\Omega$ a masa sobre la línea de enable para visualizar cuándo el puente está energizado.

---

## 5. Lista de Materiales Completa (BOM - 100% THT)

Todos los componentes son de montaje por orificio pasante y se encuentran en catálogo comercial habitual:

| Designador | Componente / Valor | Encapsulado | Función | Cant. |
| :---: | :--- | :---: | :--- | :---: |
| **U1** | **74LS175** | DIP-16 | Registro de 4 biestables D (memoria de cuadratura) | 1 |
| **U2** | **CD4052** | DIP-16 | Doble multiplexor analógico/digital 4 a 1 | 1 |
| **U3** | **L298N** | Multiwatt-15 (vertical) | Doble puente H de potencia ($2\text{A}$ por rama) | 1 |
| **U4** | **LM7805** (o MC7805CT) | TO-220 | Regulador lineal de 5V para la lógica | 1 |
| **D1..D8** | **1N5822** (Schottky 3A 40V) | DO-201AD Axial | Diodos de recirculación rápida para el motor | 8 |
| **D9** | **1N4007** (o 1N5408) | DO-41 Axial | Diodo serie de protección contra inversión de polaridad | 1 |
| **LED1** | LED 3mm o 5mm Verde | Radial | Testigo de habilitación (`ENABLE` activo) | 1 |
| **R_SENSE1,2** | **$0.50\,\Omega$ / 2W** (metal óxido) | Axial 2W | Resistencias de sensado de corriente a masa | 2 |
| **R1, R2, R3** | **$10\text{ k}\Omega$ / 0.25W** | Axial 1/4W | Resistencias de pull-up para `STEP`, `DIR` y `ENABLE` | 3 |
| **R4** | **$1\text{ k}\Omega$ / 0.25W** | Axial 1/4W | Limitadora de corriente para el LED testigo | 1 |
| **C1** | **$470\,\mu\text{F}$ / 50V** | Electrolítico radial | Filtro principal de la línea motriz ($V_S$) | 1 |
| **C2** | **$100\,\mu\text{F}$ / 35V** | Electrolítico radial | Filtro de entrada del regulador LM7805 | 1 |
| **C3** | **$10\,\mu\text{F}$ / 16V** | Electrolítico radial | Filtro de salida de 5V del LM7805 | 1 |
| **C4, C5, C6** | **$100\text{ nF}$ / 50V** | Cerámico multicapa 2.54mm | Desacoplo de alta frecuencia para U1, U2 y L298N | 3 |
| **J_PWR** | Bornera a tornillo 2 vías (KF301-2P) | Paso 5.0 mm | Alimentación motor (+12V a +24V y Masa) | 1 |
| **J_MOT** | Bornera a tornillo 4 vías (KF301-4P) | Paso 5.0 mm | Conexión a bobinas del motor bipolar | 1 |
| **J_CTRL** | Bornera a tornillo 4 vías (KF301-4P) | Paso 5.0 mm | Entradas de control (`GND`, `EN`, `DIR`, `STP`) | 1 |
| **J_SNS** | Bornera a tornillo 3 vías (KF301-3P) | Paso 5.0 mm | Derivación analógica de shunts a instrumentación | 1 |
| **JP1** | Tira de pines macho 1x03 + Jumper | Paso 2.54 mm | Selector manual de Enable (`ON` / `CTRL`) | 1 |
| **Zoc1, Zoc2** | Zócalo DIP-16 angosto | DIP-16 | Montaje seguro para 74LS175 y CD4052 | 2 |
| **HS1** | Disipador aletado de aluminio | Perfil TO-220 / Multiwatt | Disipador térmico para el L298N con tornillo M3 | 1 |

---

## 6. Guía para Simular en Falstad

En el simulador Falstad no se requieren bloques de texto personalizados ni reglas de código:

1. Colocar dos **Flip-Flops D** (`Lógica secuencial -> Flip-Flops -> Añadir Flip-Flop D`). Nombrarlos FF_A y FF_B.
2. Colocar dos **Multiplexores 2 a 1** (`Chips digitales -> Añadir multiplexor`, con 1 bit de selección). Nombrarlos MUX_A y MUX_B.
3. Conectar la señal de entrada **`DIR`** al pin de selección de ambos multiplexores.
4. Conectar la señal de entrada **`STEP`** a los pines de reloj de ambos Flip-Flops D.
5. **Realimentación de entradas:**
   - En MUX_A: conectar $\bar{Q}_B$ en la entrada 0 y $Q_B$ en la entrada 1. La salida de MUX_A va a la entrada $D$ de FF_A.
   - En MUX_B: conectar $Q_A$ en la entrada 0 y $\bar{Q}_A$ en la entrada 1. La salida de MUX_B va a la entrada $D$ de FF_B.
6. Las salidas de los biestables comandan directamente las fases:
   - $Q_A \to \text{IN1}$
   - $\bar{Q}_A \to \text{IN2}$
   - $Q_B \to \text{IN3}$
   - $\bar{Q}_B \to \text{IN4}$
7. Al aplicar pulsos en `STEP`, el circuito genera la secuencia de cuadratura completa sin necesidad de ningún estado de carga inicial.
