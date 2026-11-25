// ================================================================
// BALL AND BEAM - GEMELO DIGITAL CON ESP32
//
// CONEXION FISICA:
// 1) Conectar la ESP32 a la PC con un cable USB DE DATOS.
// 2) Revisar en Windows que puerto COM recibio (por ejemplo COM5).
// 3) Cambiar PUERTO_ESP32 mas abajo si el numero es diferente.
// 4) Cerrar PlatformIO Serial Monitor / Arduino Serial Monitor antes
//    de abrir este programa: Windows solo permite que un proceso
//    tenga abierto el mismo puerto COM a la vez.
// 5) NO es necesario cambiar el firmware del estabilizador.
//
// PROTOCOLO ACTUAL (paquete 2026-11-23):
// - Al abrir el COM se envia: STOP, APP y STATUS.
// - La ESP32 debe responder:
//       # APP READY v=1 min=2 max=31 latido_ms=1000
// - F5 envia la referencia actual con SP y despues RUN.
// - Mientras RUN esta armado, raylib envia PING cada 250 ms.
// - F6 envia STOP y detiene los PING.
// - Durante RUN la telemetria ya incluye ang y u, por lo que el gemelo
//   puede seguir servo y carrito sin modificar el firmware.
// - Al cerrar el programa se envia STOP antes de cerrar el puerto.
//
// IMPORTANTE:
// No usar START para esta integracion. El protocolo nuevo usa RUN.
// ================================================================

#define WIN32_LEAN_AND_MEAN
#define NOGDI
#define NOUSER
#include <windows.h>

#include "raylib.h"
#include "raymath.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// CODIGO CON GRAFICOS BUENOS PERO CON EL PROBLEMA DE LA NUBE BLANCA 

// ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
// CONFIGURACION GENERAL
// ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

#define ANCHO 1400
#define ALTO 850

// ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
// CONEXION ESP32 / USB SERIAL
// ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

// Cambia solamente esta linea si Windows asigna otro puerto.
#define PUERTO_ESP32 "COM5"
#define BAUD_ESP32 115200

// El protocolo de aplicacion nuevo exige un latido mientras RUN esta
// armado. El firmware aborta si pasa mas de 1 segundo sin PING.
// Enviamos PING cada 250 ms, igual que la interfaz oficial entregada.
//
// Este temporizador usa GetTime(): si raylib se congela o deja de
// ejecutar su ciclo, tambien dejan de salir PING. Eso es intencional,
// porque la ESP32 debe entrar en proteccion en ese caso.
#define PERIODO_PING_S    0.25
#define REINTENTO_COM_S   2.00

// El firmware usa 83 grados como neutro. En la estructura mecanica
// real existe aproximadamente un desfase de 7 grados y la manivela
// se ve horizontal en ese punto. Nuestro modelo 3D considera 90 grados
// como la posicion horizontal, por eso:
//
//     83 grados reales + 7 grados = 90 grados visuales
//
// Este desfase SOLO se usa para dibujar. El dato original recibido
// desde la ESP32 no se modifica.
#define SERVO_NEUTRO_ESP32       83.0f
#define DESFASE_SERVO_VISUAL      7.0f

// El firmware entrega pos en centimetros ya calibrados.
// La documentacion entregada define 0..34 cm como recorrido util
// del CENTRO del carrito.
#define POSICION_MIN_CM            0.0f
#define POSICION_MAX_CM           34.0f

#define ARCHIVO_BASE "glb/Base.glb"
#define ARCHIVO_HINGE_TOWER "glb/Hinge_Tower.glb"
#define ARCHIVO_SERVO "glb/Servo_MG996R.glb"
#define ARCHIVO_BASE_SERVO "glb/Base_de_Servo.glb"
#define ARCHIVO_CRANK "glb/Crank.glb"
#define ARCHIVO_COUPLER "glb/Coupler.glb"
#define ARCHIVO_DOWEL "glb/Dowel.glb"
#define ARCHIVO_BEAM_END "glb/Beam_End.glb"
#define ARCHIVO_BEAM_HINGE "glb/Beam_Hinge.glb"
#define ARCHIVO_BEARING "glb/605ZZ_Bearing.glb"
#define ARCHIVO_SENSOR "glb/GYUL53L0X.glb"
#define ARCHIVO_CART "glb/Cart.glb"
#define ARCHIVO_REGLA "glb/Regla.glb"


// ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
// COLORES
// ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

#define FONDO_SUPERIOR \
    (Color){225,230,232,255}

#define FONDO_INFERIOR \
    (Color){177,186,190,255}

#define PISO_COLOR \
    (Color){190,195,197,255}

#define PANEL_COLOR \
    (Color){24,29,31,225}

#define VERDE \
    (Color){0,142,96,255}

#define TEXTO_CLARO \
    (Color){238,242,243,255}


// ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
// LECTOR SERIAL ESP32
//
// La ESP32 entrega dos tipos de informacion que nos interesan:
//
// TELEMETRIA NORMAL:
// Ref: 17.0 | pos: 15.8 | err: 1.2 | Estado: LQI
//
// RESPUESTA A STATUS:
// # K=[...] Ki=... u=... angulo_orden=83.00
//
// Durante ensayos DE tambien puede llegar:
// Ref: ... | pos: ... | err: ... | Estado: ... | ang: 83.00 | u: ...
//
// Raylib usa pos para mover el carrito y angulo_orden/ang para mover
// el mecanismo. No se simula el sensor ni el controlador dentro de
// raylib: el sistema fisico sigue siendo la fuente de verdad.
// ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

typedef struct ConexionESP32
{
    HANDLE puerto;
    bool conectado;

    char linea[256];
    int longitudLinea;

    float referenciaCm;
    float posicionCm;
    float errorCm;
    float anguloOrdenGrados;
    float uRadS2;

    bool tienePosicion;
    bool tieneAngulo;

    // Estado del protocolo de aplicacion del firmware 2026-11-23.
    bool appReady;
    bool controlArmado;
    bool latidoActivo;
    bool calibracionCargada;

    char estado[32];
    char ultimoMensaje[160];

    double ultimoDatoS;
    double ultimoAnguloS;
    double ultimoPingS;
    double proximoReintentoS;

} ConexionESP32;


void InicializarDatosESP32(ConexionESP32 *esp32)
{
    memset(esp32, 0, sizeof(*esp32));

    esp32->puerto = INVALID_HANDLE_VALUE;
    esp32->referenciaCm = 17.0f;
    esp32->posicionCm = 17.0f;
    esp32->errorCm = 0.0f;
    esp32->anguloOrdenGrados = SERVO_NEUTRO_ESP32;
    esp32->uRadS2 = 0.0f;

    strcpy(esp32->estado, "Sin datos");
    strcpy(esp32->ultimoMensaje, "Esperando conexion");
    esp32->ultimoPingS = -1000.0;
}


void CerrarESP32(ConexionESP32 *esp32)
{
    if(esp32->puerto != INVALID_HANDLE_VALUE)
    {
        // El protocolo nuevo exige detener el control antes de cerrar
        // o desconectar la aplicacion.
        if(esp32->conectado)
        {
            const char *stop = "STOP\n";
            DWORD enviados = 0;

            WriteFile(
                esp32->puerto,
                stop,
                (DWORD)strlen(stop),
                &enviados,
                NULL
            );
        }

        CloseHandle(esp32->puerto);
    }

    esp32->puerto = INVALID_HANDLE_VALUE;
    esp32->conectado = false;
    esp32->longitudLinea = 0;
    esp32->appReady = false;
    esp32->controlArmado = false;
    esp32->latidoActivo = false;
}


bool AbrirESP32(ConexionESP32 *esp32, const char *nombrePuerto)
{
    CerrarESP32(esp32);

    char rutaPuerto[64];

    // La forma \\.\COMx funciona tambien con COM10, COM11, etc.
    snprintf(
        rutaPuerto,
        sizeof(rutaPuerto),
        "\\\\.\\%s",
        nombrePuerto
    );

    HANDLE h = CreateFileA(
        rutaPuerto,
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL
    );

    if(h == INVALID_HANDLE_VALUE)
    {
        return false;
    }

    DCB dcb;
    memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);

    if(!GetCommState(h, &dcb))
    {
        CloseHandle(h);
        return false;
    }

    dcb.BaudRate = BAUD_ESP32;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;

    // No necesitamos DTR/RTS para la telemetria. Deshabilitarlos reduce
    // la posibilidad de que el adaptador USB reinicie la ESP32 al abrir.
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;

    if(!SetCommState(h, &dcb))
    {
        CloseHandle(h);
        return false;
    }

    COMMTIMEOUTS tiempos;
    memset(&tiempos, 0, sizeof(tiempos));

    // Lectura no bloqueante: ReadFile regresa inmediatamente con los
    // bytes que ya esten disponibles. La ventana de raylib nunca espera
    // a que llegue una nueva linea de la ESP32.
    tiempos.ReadIntervalTimeout = MAXDWORD;
    tiempos.ReadTotalTimeoutMultiplier = 0;
    tiempos.ReadTotalTimeoutConstant = 0;
    tiempos.WriteTotalTimeoutMultiplier = 0;
    tiempos.WriteTotalTimeoutConstant = 50;

    if(!SetCommTimeouts(h, &tiempos))
    {
        CloseHandle(h);
        return false;
    }

    SetupComm(h, 4096, 4096);
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);

    esp32->puerto = h;
    esp32->conectado = true;
    esp32->longitudLinea = 0;

    esp32->appReady = false;
    esp32->controlArmado = false;
    esp32->latidoActivo = false;
    esp32->calibracionCargada = false;
    esp32->ultimoPingS = -1000.0;

    strcpy(
        esp32->ultimoMensaje,
        "COM abierto; negociando APP"
    );

    // Secuencia oficial de apertura indicada en la entrega nueva:
    //
    // STOP   -> asegurar que la planta quede en neutro
    // APP    -> negociar el protocolo de aplicacion
    // STATUS -> obtener diagnostico inicial y angulo de orden
    //
    // Se mandan en este orden y NO se usa START.
    {
        const char *inicio =
            "STOP\n"
            "APP\n"
            "STATUS\n";

        DWORD enviados = 0;

        if(!WriteFile(
            esp32->puerto,
            inicio,
            (DWORD)strlen(inicio),
            &enviados,
            NULL
        ))
        {
            CerrarESP32(esp32);
            return false;
        }
    }

    return true;
}


bool EnviarESP32(ConexionESP32 *esp32, const char *texto)
{
    if(!esp32->conectado || esp32->puerto == INVALID_HANDLE_VALUE)
    {
        return false;
    }

    DWORD enviados = 0;
    DWORD longitud = (DWORD)strlen(texto);

    if(!WriteFile(esp32->puerto, texto, longitud, &enviados, NULL))
    {
        CerrarESP32(esp32);
        return false;
    }

    return enviados == longitud;
}


void LimpiarEstadoTexto(char *texto)
{
    size_t n = strlen(texto);

    while(n > 0 &&
          (texto[n - 1] == ' ' || texto[n - 1] == '\t'))
    {
        texto[n - 1] = '\0';
        n--;
    }
}


void ProcesarLineaESP32(ConexionESP32 *esp32, const char *linea)
{
    if(linea == NULL || linea[0] == '\0')
    {
        return;
    }

    // Guardar el ultimo mensaje de diagnostico de la ESP32.
    // Las lineas de telemetria se procesan mas abajo.
    if(linea[0] == '#')
    {
        strncpy(
            esp32->ultimoMensaje,
            linea,
            sizeof(esp32->ultimoMensaje) - 1
        );

        esp32->ultimoMensaje[
            sizeof(esp32->ultimoMensaje) - 1
        ] = '\0';
    }


    // Handshake exacto del protocolo de aplicacion.
    if(strcmp(
        linea,
        "# APP READY v=1 min=2 max=31 latido_ms=1000"
    ) == 0)
    {
        esp32->appReady = true;
    }


    if(strstr(linea, "# APP CONTROL ARMADO") != NULL)
    {
        esp32->controlArmado = true;
        esp32->latidoActivo = true;
        esp32->ultimoPingS = -1000.0;
    }


    // Cualquier proteccion o pausa corta el latido local.
    if(strstr(linea, "# APP ABORT") != NULL ||
       strstr(linea, "# APP DETENIDO") != NULL ||
       strstr(linea, "# APP ERROR") != NULL ||
       strstr(linea, "# Control pausado") != NULL ||
       strstr(linea, "# DE ABORT") != NULL)
    {
        esp32->controlArmado = false;
        esp32->latidoActivo = false;
    }


    if(strstr(linea, "# Calibracion NVS cargada") != NULL ||
       strstr(linea, "# Calibracion guardada aplicada") != NULL)
    {
        esp32->calibracionCargada = true;
    }


    if(strstr(linea, "# Falta calibracion NVS") != NULL ||
       strstr(linea, "# Sin calibracion") != NULL)
    {
        esp32->calibracionCargada = false;
    }


    double ref = 0.0;
    double pos = 0.0;
    double err = 0.0;
    double ang = 0.0;
    double u = 0.0;
    char estado[32] = {0};

    int n = sscanf(
        linea,
        "Ref: %lf | pos: %lf | err: %lf | Estado: %31[^|\r\n] | ang: %lf | u: %lf",
        &ref,
        &pos,
        &err,
        estado,
        &ang,
        &u
    );

    // La misma expresion devuelve 4 cuando llega la telemetria normal
    // y 6 cuando un ensayo DE incluye ang y u.
    if(n == 4 || n == 6)
    {
        if(isfinite(ref) && isfinite(pos) && isfinite(err))
        {
            esp32->referenciaCm = (float)ref;
            esp32->posicionCm = (float)pos;
            esp32->errorCm = (float)err;
            esp32->tienePosicion = true;
            esp32->ultimoDatoS = GetTime();

            strncpy(esp32->estado, estado, sizeof(esp32->estado) - 1);
            esp32->estado[sizeof(esp32->estado) - 1] = '\0';
            LimpiarEstadoTexto(esp32->estado);
        }

        if(n == 6 && isfinite(ang) && isfinite(u))
        {
            esp32->anguloOrdenGrados = (float)ang;
            esp32->uRadS2 = (float)u;
            esp32->tieneAngulo = true;
            esp32->ultimoAnguloS = GetTime();
        }

        return;
    }

    // STATUS produce una linea que empieza con # y contiene
    // "angulo_orden=83.00". No necesitamos copiar K ni Ki para
    // animar el gemelo; solo extraemos ese campo.
    const char *marcaAngulo = strstr(linea, "angulo_orden=");

    if(marcaAngulo != NULL)
    {
        char *fin = NULL;
        double valor = strtod(marcaAngulo + strlen("angulo_orden="), &fin);

        if(fin != marcaAngulo + strlen("angulo_orden=") && isfinite(valor))
        {
            esp32->anguloOrdenGrados = (float)valor;
            esp32->tieneAngulo = true;
            esp32->ultimoAnguloS = GetTime();
        }
    }
}


void LeerESP32(ConexionESP32 *esp32)
{
    if(!esp32->conectado || esp32->puerto == INVALID_HANDLE_VALUE)
    {
        return;
    }

    DWORD errores = 0;
    COMSTAT estadoPuerto;

    if(!ClearCommError(esp32->puerto, &errores, &estadoPuerto))
    {
        CerrarESP32(esp32);
        return;
    }

    while(estadoPuerto.cbInQue > 0)
    {
        char bloque[128];
        DWORD leidos = 0;
        DWORD pedir = estadoPuerto.cbInQue;

        if(pedir > sizeof(bloque))
        {
            pedir = sizeof(bloque);
        }

        if(!ReadFile(esp32->puerto, bloque, pedir, &leidos, NULL))
        {
            CerrarESP32(esp32);
            return;
        }

        if(leidos == 0)
        {
            break;
        }

        for(DWORD i = 0; i < leidos; i++)
        {
            char c = bloque[i];

            if(c == '\r')
            {
                continue;
            }

            if(c == '\n')
            {
                if(esp32->longitudLinea > 0)
                {
                    esp32->linea[esp32->longitudLinea] = '\0';
                    ProcesarLineaESP32(esp32, esp32->linea);
                    esp32->longitudLinea = 0;
                }

                continue;
            }

            if(esp32->longitudLinea < (int)sizeof(esp32->linea) - 1)
            {
                esp32->linea[esp32->longitudLinea++] = c;
            }
            else
            {
                // Si por ruido llega una linea demasiado larga, se descarta
                // y se espera el siguiente salto de linea.
                esp32->longitudLinea = 0;
            }
        }

        if(!ClearCommError(esp32->puerto, &errores, &estadoPuerto))
        {
            CerrarESP32(esp32);
            return;
        }
    }
}


void ActualizarConexionESP32(ConexionESP32 *esp32)
{
    double ahora = GetTime();

    if(!esp32->conectado)
    {
        if(ahora >= esp32->proximoReintentoS)
        {
            if(!AbrirESP32(esp32, PUERTO_ESP32))
            {
                esp32->proximoReintentoS = ahora + REINTENTO_COM_S;
            }
        }

        return;
    }

    LeerESP32(esp32);

    // Durante RUN el protocolo exige PING continuo.
    //
    // No se liga a un contador de frames: usamos tiempo real.
    // Si el ciclo de raylib se bloquea, los PING se detienen y la
    // proteccion de la ESP32 puede actuar como fue disenada.
    if(esp32->latidoActivo &&
       ahora - esp32->ultimoPingS >= PERIODO_PING_S)
    {
        if(EnviarESP32(esp32, "PING\n"))
        {
            esp32->ultimoPingS = ahora;
        }
    }
}


// ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
// LIMITAR VALOR
// ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

float LimitarFloat(
    float valor,
    float minimo,
    float maximo
)
{
    if(valor < minimo)
    {
        return minimo;
    }

    if(valor > maximo)
    {
        return maximo;
    }

    return valor;
}



// ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
// CINEMATICA DEL BALL AND BEAM
// ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

#define PIVOTE_MOTOR_X  0.1833599955f
#define PIVOTE_MOTOR_Y  0.0718000010f

#define PIVOTE_BEAM_X  -0.2134999931f
#define PIVOTE_BEAM_Y   0.1626999974f

#define RADIO_CRANK     0.0310000000f
#define LARGO_COUPLER   0.0900000000f
#define LARGO_BEAM      0.4180000000f

#define ANGULO_CRANK_INICIAL    2.65886191f
#define ANGULO_COUPLER_INICIAL 96.26831468f
#define ANGULO_BEAM_INICIAL     0.00000000f

// Recorrido seguro del centro del carrito sobre el eje del beam.
//
// Los extremos físicos del beam están aproximadamente entre
// X = -0.2135 m y X = 0.1835 m.
// Se deja margen para que el cuerpo del carrito no se salga
// ni choque con los soportes de los extremos.

#define CART_X_MIN             -0.1650000000f
#define CART_X_MAX              0.1350000000f
#define CART_Y_EJE              0.1742208749f
#define CART_Z_EJE              0.0024000001f


Vector3 RotarPuntoZ(
    Vector3 punto,
    Vector2 pivote,
    float anguloGrados
)
{
    float a =
        anguloGrados *
        DEG2RAD;


    float c =
        cosf(a);


    float s =
        sinf(a);


    float x =
        punto.x -
        pivote.x;


    float y =
        punto.y -
        pivote.y;


    return
        (Vector3)
        {
            pivote.x +
            x*c -
            y*s,

            pivote.y +
            x*s +
            y*c,

            punto.z
        };
}


bool CalcularMecanismo(
    float anguloMotor,
    Vector2 *puntoCrank,
    Vector2 *puntoBeam,
    float *anguloCoupler,
    float *anguloBeam
)
{
    // Referencia angular corregida:
    // 0 grados   = crank hacia arriba
    // 90 grados  = crank hacia la derecha
    // 180 grados = crank hacia abajo
    //
    // De esta forma el crank recorre el semicirculo derecho.

    float anguloCrankVisual =
        90.0f -
        anguloMotor;


    float theta =
        anguloCrankVisual *
        DEG2RAD;


    puntoCrank->x =
        PIVOTE_MOTOR_X +
        RADIO_CRANK *
        cosf(theta);


    puntoCrank->y =
        PIVOTE_MOTOR_Y +
        RADIO_CRANK *
        sinf(theta);


    float dx =
        PIVOTE_BEAM_X -
        puntoCrank->x;


    float dy =
        PIVOTE_BEAM_Y -
        puntoCrank->y;


    float d =
        sqrtf(
            dx*dx +
            dy*dy
        );


    if(
        d <= 0.000001f ||
        d > LARGO_COUPLER + LARGO_BEAM ||
        d < fabsf(
            LARGO_COUPLER -
            LARGO_BEAM
        )
    )
    {
        return false;
    }


    float a =
        (
            LARGO_COUPLER*LARGO_COUPLER -
            LARGO_BEAM*LARGO_BEAM +
            d*d
        ) /
        (
            2.0f*d
        );


    float h2 =
        LARGO_COUPLER*LARGO_COUPLER -
        a*a;


    if(
        h2 <
        0.0f
    )
    {
        h2 =
            0.0f;
    }


    float h =
        sqrtf(h2);


    float ux =
        dx/d;


    float uy =
        dy/d;


    float baseX =
        puntoCrank->x +
        a*ux;


    float baseY =
        puntoCrank->y +
        a*uy;


    Vector2 solucionA =
    {
        baseX -
        h*uy,

        baseY +
        h*ux
    };


    Vector2 solucionB =
    {
        baseX +
        h*uy,

        baseY -
        h*ux
    };


    if(
        solucionA.y >
        solucionB.y
    )
    {
        *puntoBeam =
            solucionA;
    }
    else
    {
        *puntoBeam =
            solucionB;
    }


    *anguloCoupler =
        atan2f(
            puntoBeam->y -
            puntoCrank->y,

            puntoBeam->x -
            puntoCrank->x
        ) *
        RAD2DEG;


    *anguloBeam =
        atan2f(
            puntoBeam->y -
            PIVOTE_BEAM_Y,

            puntoBeam->x -
            PIVOTE_BEAM_X
        ) *
        RAD2DEG;


    return true;
}


Quaternion RotacionExtraZ(
    Quaternion rotacionOriginal,
    float anguloExtraGrados
)
{
    Quaternion extra =
        QuaternionFromAxisAngle(
            (Vector3)
            {
                0.0f,
                0.0f,
                1.0f
            },

            anguloExtraGrados *
            DEG2RAD
        );


    return
        QuaternionNormalize(
            QuaternionMultiply(
                extra,
                rotacionOriginal
            )
        );
}


void DibujarPieza(
    Model modelo,
    Vector3 posicionLocal,
    Quaternion rotacionLocal,
    Vector3 escalaLocal,
    float escalaGlobal,
    float giroGlobalY,
    Vector3 posicionGlobal
)
{
    Quaternion giroGlobal =
        QuaternionFromAxisAngle(
            (Vector3)
            {
                0.0f,
                1.0f,
                0.0f
            },

            giroGlobalY *
            DEG2RAD
        );


    Vector3 posicionEscalada =
        Vector3Scale(
            posicionLocal,
            escalaGlobal
        );


    Vector3 posicionRotada =
        Vector3RotateByQuaternion(
            posicionEscalada,
            giroGlobal
        );


    Vector3 posicionFinal =
        Vector3Add(
            posicionGlobal,
            posicionRotada
        );


    Quaternion rotacionFinal =
        QuaternionNormalize(
            QuaternionMultiply(
                giroGlobal,
                rotacionLocal
            )
        );


    Vector3 eje =
    {
        0.0f,
        1.0f,
        0.0f
    };


    float angulo =
        0.0f;


    QuaternionToAxisAngle(
        rotacionFinal,
        &eje,
        &angulo
    );


    Vector3 escalaFinal =
    {
        escalaLocal.x *
        escalaGlobal,

        escalaLocal.y *
        escalaGlobal,

        escalaLocal.z *
        escalaGlobal
    };


    DrawModelEx(
        modelo,
        posicionFinal,
        eje,
        angulo *
        RAD2DEG,
        escalaFinal,
        WHITE
    );
}


void AplicarShaderModelo(
    Model *modelo,
    Shader shader
)
{
    for(
        int i = 0;
        i < modelo->materialCount;
        i++
    )
    {
        modelo->materials[i].shader =
            shader;
    }
}


// ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
// VERTEX SHADER
// ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

static const char *VERTEX_SHADER =
"#version 330\n"

"in vec3 vertexPosition;\n"
"in vec2 vertexTexCoord;\n"
"in vec3 vertexNormal;\n"
"in vec4 vertexColor;\n"

"uniform mat4 mvp;\n"
"uniform mat4 matModel;\n"
"uniform mat4 matNormal;\n"

"out vec3 fragPos;\n"
"out vec2 fragTexCoord;\n"
"out vec3 fragNormal;\n"
"out vec4 fragColor;\n"

"void main()\n"
"{\n"

"    vec4 worldPosition =\n"
"        matModel *\n"
"        vec4(vertexPosition, 1.0);\n"

"    fragPos =\n"
"        worldPosition.xyz;\n"

"    fragTexCoord =\n"
"        vertexTexCoord;\n"

"    fragNormal =\n"
"        normalize(\n"
"            vec3(\n"
"                matNormal *\n"
"                vec4(vertexNormal, 0.0)\n"
"            )\n"
"        );\n"

"    fragColor =\n"
"        vertexColor;\n"

"    gl_Position =\n"
"        mvp *\n"
"        vec4(vertexPosition, 1.0);\n"

"}\n";


// ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
// FRAGMENT SHADER
// ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

static const char *FRAGMENT_SHADER =
"#version 330\n"

"in vec3 fragPos;\n"
"in vec2 fragTexCoord;\n"
"in vec3 fragNormal;\n"
"in vec4 fragColor;\n"

"uniform sampler2D texture0;\n"
"uniform vec4 colDiffuse;\n"

"uniform vec3 viewPos;\n"

"uniform vec3 lightDir1;\n"
"uniform vec3 lightDir2;\n"

"uniform vec3 lightColor1;\n"
"uniform vec3 lightColor2;\n"

"uniform vec3 ambientColor;\n"

"uniform float ambientStrength;\n"
"uniform float fillStrength;\n"
"uniform float specularStrength;\n"
"uniform float shininess;\n"

"out vec4 finalColor;\n"

"void main()\n"
"{\n"

"    vec4 texel =\n"
"        texture(\n"
"            texture0,\n"
"            fragTexCoord\n"
"        ) *\n"
"        colDiffuse *\n"
"        fragColor;\n"


"    vec3 N =\n"
"        normalize(\n"
"            fragNormal\n"
"        );\n"


"    vec3 L1 =\n"
"        normalize(\n"
"            -lightDir1\n"
"        );\n"


"    vec3 L2 =\n"
"        normalize(\n"
"            -lightDir2\n"
"        );\n"


"    vec3 V =\n"
"        normalize(\n"
"            viewPos - fragPos\n"
"        );\n"


"    float diffuse1 =\n"
"        max(\n"
"            dot(N, L1),\n"
"            0.0\n"
"        );\n"


"    float diffuse2 =\n"
"        max(\n"
"            dot(N, L2),\n"
"            0.0\n"
"        );\n"


"    vec3 H1 =\n"
"        normalize(\n"
"            L1 + V\n"
"        );\n"


"    float specular =\n"
"        pow(\n"
"            max(\n"
"                dot(N, H1),\n"
"                0.0\n"
"            ),\n"
"            shininess\n"
"        ) *\n"
"        specularStrength;\n"


"    vec3 ambient =\n"
"        ambientColor *\n"
"        ambientStrength;\n"


"    vec3 direct =\n"
"        lightColor1 *\n"
"        diffuse1;\n"


"    vec3 fill =\n"
"        lightColor2 *\n"
"        diffuse2 *\n"
"        fillStrength;\n"


"    vec3 color =\n"
"        texel.rgb *\n"
"        (\n"
"            ambient +\n"
"            direct +\n"
"            fill\n"
"        );\n"


"    color +=\n"
"        lightColor1 *\n"
"        specular;\n"


"    color =\n"
"        color /\n"
"        (\n"
"            color +\n"
"            vec3(0.65)\n"
"        );\n"


"    color =\n"
"        pow(\n"
"            color,\n"
"            vec3(1.0 / 2.2)\n"
"        );\n"


"    finalColor =\n"
"        vec4(\n"
"            color,\n"
"            texel.a\n"
"        );\n"

"}\n";


// ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
// MAIN
// ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

int main(void)
{
    SetConfigFlags(
        FLAG_MSAA_4X_HINT |
        FLAG_WINDOW_RESIZABLE
    );


    InitWindow(
        ANCHO,
        ALTO,
        "Ball and Beam - Visor 3D"
    );


    SetTargetFPS(
        60
    );


    // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
    // VERIFICAR MODELOS
    // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

    bool archivosCorrectos =
        FileExists(ARCHIVO_BASE) &&
        FileExists(ARCHIVO_HINGE_TOWER) &&
        FileExists(ARCHIVO_SERVO) &&
        FileExists(ARCHIVO_BASE_SERVO) &&
        FileExists(ARCHIVO_CRANK) &&
        FileExists(ARCHIVO_COUPLER) &&
        FileExists(ARCHIVO_DOWEL) &&
        FileExists(ARCHIVO_BEAM_END) &&
        FileExists(ARCHIVO_BEAM_HINGE) &&
        FileExists(ARCHIVO_BEARING) &&
        FileExists(ARCHIVO_SENSOR) &&
        FileExists(ARCHIVO_CART) &&
        FileExists(ARCHIVO_REGLA);


    if(
        !archivosCorrectos
    )
    {
        while(
            !WindowShouldClose()
        )
        {
            BeginDrawing();


            ClearBackground(
                RAYWHITE
            );


            DrawText(
                "FALTAN ARCHIVOS .GLB",
                50,
                60,
                30,
                RED
            );


            DrawText(
                "Coloca la carpeta glb completa junto al .exe",
                50,
                110,
                20,
                DARKGRAY
            );


            EndDrawing();
        }


        CloseWindow();

        return 1;
    }


    // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
    // CARGAR PIEZAS DIRECTAS DE SOLIDWORKS
    // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

    Model modeloBase =
        LoadModel(
            ARCHIVO_BASE
        );


    Model modeloHingeTower =
        LoadModel(
            ARCHIVO_HINGE_TOWER
        );


    Model modeloServo =
        LoadModel(
            ARCHIVO_SERVO
        );


    Model modeloBaseServo =
        LoadModel(
            ARCHIVO_BASE_SERVO
        );


    Model modeloCrank =
        LoadModel(
            ARCHIVO_CRANK
        );


    Model modeloCoupler =
        LoadModel(
            ARCHIVO_COUPLER
        );


    Model modeloDowel =
        LoadModel(
            ARCHIVO_DOWEL
        );


    Model modeloBeamEnd =
        LoadModel(
            ARCHIVO_BEAM_END
        );


    Model modeloBeamHinge =
        LoadModel(
            ARCHIVO_BEAM_HINGE
        );


    Model modeloBearing =
        LoadModel(
            ARCHIVO_BEARING
        );


    Model modeloSensor =
        LoadModel(
            ARCHIVO_SENSOR
        );


    Model modeloCart =
        LoadModel(
            ARCHIVO_CART
        );


    Model modeloRegla =
        LoadModel(
            ARCHIVO_REGLA
        );


    // El ensamble original mide 0.635 m en su dimension mayor.
    // Esto conserva exactamente la escala visual que usaba Claude.

    float dimensionMayor =
        0.635f;


    float escalaInicial =
        5.5f /
        dimensionMayor;


    float escala =
        escalaInicial;


    // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
    // CARGAR SHADER
    // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

    Shader shader =
        LoadShaderFromMemory(
            VERTEX_SHADER,
            FRAGMENT_SHADER
        );


    shader.locs[
        SHADER_LOC_VECTOR_VIEW
    ] =
        GetShaderLocation(
            shader,
            "viewPos"
        );


    shader.locs[
        SHADER_LOC_MATRIX_MODEL
    ] =
        GetShaderLocation(
            shader,
            "matModel"
        );


    shader.locs[
        SHADER_LOC_MATRIX_NORMAL
    ] =
        GetShaderLocation(
            shader,
            "matNormal"
        );


    shader.locs[
        SHADER_LOC_MAP_DIFFUSE
    ] =
        GetShaderLocation(
            shader,
            "texture0"
        );


    shader.locs[
        SHADER_LOC_COLOR_DIFFUSE
    ] =
        GetShaderLocation(
            shader,
            "colDiffuse"
        );


    // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
    // VARIABLES DEL SHADER
    // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

    int locLightDir1 =
        GetShaderLocation(
            shader,
            "lightDir1"
        );


    int locLightDir2 =
        GetShaderLocation(
            shader,
            "lightDir2"
        );


    int locLightColor1 =
        GetShaderLocation(
            shader,
            "lightColor1"
        );


    int locLightColor2 =
        GetShaderLocation(
            shader,
            "lightColor2"
        );


    int locAmbientColor =
        GetShaderLocation(
            shader,
            "ambientColor"
        );


    int locAmbientStrength =
        GetShaderLocation(
            shader,
            "ambientStrength"
        );


    int locFillStrength =
        GetShaderLocation(
            shader,
            "fillStrength"
        );


    int locSpecularStrength =
        GetShaderLocation(
            shader,
            "specularStrength"
        );


    int locShininess =
        GetShaderLocation(
            shader,
            "shininess"
        );


    // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
    // ILUMINACION
    // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

    Vector3 lightDir1 =
        Vector3Normalize(
            (Vector3)
            {
                -0.55f,
                -1.0f,
                -0.35f
            }
        );


    Vector3 lightDir2 =
        Vector3Normalize(
            (Vector3)
            {
                0.65f,
                -0.45f,
                0.70f
            }
        );


    // Luz principal ligeramente calida.

    float lightColor1[3] =
    {
        1.00f,
        0.96f,
        0.90f
    };


    // Luz secundaria ligeramente fria.

    float lightColor2[3] =
    {
        0.50f,
        0.60f,
        0.72f
    };


    float ambientColor[3] =
    {
        0.78f,
        0.82f,
        0.85f
    };


    float ambientStrength =
        0.52f;


    float fillStrength =
        0.32f;


    float specularStrength =
        0.45f;


    float shininess =
        48.0f;


    SetShaderValue(
        shader,
        locLightDir1,
        &lightDir1,
        SHADER_UNIFORM_VEC3
    );


    SetShaderValue(
        shader,
        locLightDir2,
        &lightDir2,
        SHADER_UNIFORM_VEC3
    );


    SetShaderValue(
        shader,
        locLightColor1,
        lightColor1,
        SHADER_UNIFORM_VEC3
    );


    SetShaderValue(
        shader,
        locLightColor2,
        lightColor2,
        SHADER_UNIFORM_VEC3
    );


    SetShaderValue(
        shader,
        locAmbientColor,
        ambientColor,
        SHADER_UNIFORM_VEC3
    );


    SetShaderValue(
        shader,
        locAmbientStrength,
        &ambientStrength,
        SHADER_UNIFORM_FLOAT
    );


    SetShaderValue(
        shader,
        locFillStrength,
        &fillStrength,
        SHADER_UNIFORM_FLOAT
    );


    SetShaderValue(
        shader,
        locSpecularStrength,
        &specularStrength,
        SHADER_UNIFORM_FLOAT
    );


    SetShaderValue(
        shader,
        locShininess,
        &shininess,
        SHADER_UNIFORM_FLOAT
    );


    // Aplicar EXACTAMENTE el mismo shader de Claude
    // a los materiales originales de cada pieza.

    AplicarShaderModelo(&modeloBase, shader);
    AplicarShaderModelo(&modeloHingeTower, shader);
    AplicarShaderModelo(&modeloServo, shader);
    AplicarShaderModelo(&modeloBaseServo, shader);
    AplicarShaderModelo(&modeloCrank, shader);
    AplicarShaderModelo(&modeloCoupler, shader);
    AplicarShaderModelo(&modeloDowel, shader);
    AplicarShaderModelo(&modeloBeamEnd, shader);
    AplicarShaderModelo(&modeloBeamHinge, shader);
    AplicarShaderModelo(&modeloBearing, shader);
    AplicarShaderModelo(&modeloSensor, shader);
    AplicarShaderModelo(&modeloCart, shader);
    AplicarShaderModelo(&modeloRegla, shader);


    // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
    // POSICION DEL MODELO
    // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

    Vector3 posicionModelo =
    {
        0.0f,
        0.02f,
        0.0f
    };


    float giroModeloY =
        0.0f;


    // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
    // CAMARA
    // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

    Camera3D camera =
    {
        0
    };


    Vector3 objetivoCamara =
    {
        0.0f,
        1.30f,
        0.0f
    };


    float yaw =
        0.70f;


    float pitch =
        0.32f;


    float distanciaCamara =
        8.8f;


    camera.up =
        (Vector3)
        {
            0.0f,
            1.0f,
            0.0f
        };


    camera.fovy =
        42.0f;


    camera.projection =
        CAMERA_PERSPECTIVE;


    // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
    // VELOCIDADES
    // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

    float velocidadMovimiento =
        2.0f;


    float velocidadRotacion =
        55.0f;


    bool mostrarPanel =
        true;


    bool autoRotar =
        false;


    // ------------------------------------------------------------
    // CONTROL DEL GEMELO
    // ------------------------------------------------------------
    //
    // anguloMotor es el angulo VISUAL de la manivela usado por el
    // modelo 3D. En modo ESP32 se calcula desde la orden real:
    //
    //     anguloMotor = anguloOrdenESP32 + 7 grados
    //
    // Por eso el neutro real de 83 grados aparece horizontal a 90
    // grados en el gemelo. El valor crudo de 83 NO se pierde.

    float anguloMotor =
        90.0f;


    float velocidadMotor =
        45.0f;


    // Posicion 3D persistente del carrito. En modo ESP32 se obtiene
    // de pos_cm (0..34 cm). En modo manual se conserva una relacion
    // simple con A/D solo para poder probar el modelo sin hardware.

    float posicionCartX =
        (CART_X_MIN + CART_X_MAX) * 0.5f;


    ConexionESP32 esp32;
    InicializarDatosESP32(&esp32);


    // El programa intenta conectarse automaticamente al abrirse.
    // M cambia entre modo ESP32 y modo manual.
    // C fuerza una reconexion al puerto configurado.

    bool modoESP32 =
        true;


    AbrirESP32(&esp32, PUERTO_ESP32);


    // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
    // CICLO PRINCIPAL
    // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

    while(
        !WindowShouldClose()
    )
    {
        float dt =
            GetFrameTime();


        Vector2 deltaMouse =
            GetMouseDelta();


        // La lectura serial se hace una vez por frame pero nunca bloquea.
        // Si la ESP32 se desconecta, se conserva el ultimo estado visual
        // y el programa intenta reconectarse automaticamente.

        ActualizarConexionESP32(&esp32);


        if(IsKeyPressed(KEY_M))
        {
            modoESP32 =
                !modoESP32;
        }


        if(IsKeyPressed(KEY_C))
        {
            CerrarESP32(&esp32);
            esp32.proximoReintentoS = 0.0;
            AbrirESP32(&esp32, PUERTO_ESP32);
        }


        // PROTOCOLO NUEVO DE LA APLICACION:
        //
        // F5:
        // 1) Reenvia a la ESP32 la referencia que ella misma esta
        //    reportando actualmente. Esto NO cambia el setpoint.
        // 2) Envia RUN.
        // 3) Activa PING cada 250 ms mientras el supervisor este armado.
        //
        // F6:
        // Envia STOP, detiene los PING y lleva el servo al neutro.
        //
        // START ya NO se usa para la integracion con la laptop.

        if(IsKeyPressed(KEY_F5) && esp32.conectado)
        {
            if(esp32.appReady)
            {
                char comandoSP[48];

                float referenciaSegura =
                    LimitarFloat(
                        esp32.referenciaCm,
                        2.0f,
                        31.0f
                    );


                snprintf(
                    comandoSP,
                    sizeof(comandoSP),
                    "SP %.2f\n",
                    referenciaSegura
                );


                if(EnviarESP32(&esp32, comandoSP) &&
                   EnviarESP32(&esp32, "RUN\n"))
                {
                    // Empezamos a latir inmediatamente. Si RUN es rechazado,
                    // la respuesta APP ERROR desactivara este estado al
                    // procesarse.
                    esp32.latidoActivo = true;
                    esp32.ultimoPingS = -1000.0;

                    strcpy(
                        esp32.ultimoMensaje,
                        "RUN enviado; esperando APP CONTROL ARMADO"
                    );
                }
            }
            else
            {
                // Si aun no llego APP READY, volver a negociar sin arrancar.
                EnviarESP32(&esp32, "APP\n");
                EnviarESP32(&esp32, "STATUS\n");

                strcpy(
                    esp32.ultimoMensaje,
                    "Esperando APP READY; RUN no enviado"
                );
            }
        }


        if(IsKeyPressed(KEY_F6) && esp32.conectado)
        {
            EnviarESP32(&esp32, "STOP\n");
            esp32.latidoActivo = false;
            esp32.controlArmado = false;
        }


        // F7 permite pedir un STATUS manual cuando se quiera revisar
        // el angulo de orden sin inundar el puerto durante el control.
        if(IsKeyPressed(KEY_F7) && esp32.conectado)
        {
            EnviarESP32(&esp32, "STATUS\n");
        }


        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
        // ORBITAR CAMARA
        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

        if(
            IsMouseButtonDown(
                MOUSE_BUTTON_LEFT
            )
        )
        {
            yaw -=
                deltaMouse.x *
                0.005f;


            pitch -=
                deltaMouse.y *
                0.005f;
        }


        pitch =
            LimitarFloat(
                pitch,
                -0.25f,
                1.20f
            );


        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
        // ZOOM
        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

        distanciaCamara -=
            GetMouseWheelMove() *
            0.65f;


        distanciaCamara =
            LimitarFloat(
                distanciaCamara,
                3.2f,
                18.0f
            );


        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
        // MOVER MODELO EN LOS EJES X / Z
        //
        // Flecha izquierda / derecha = eje X
        // Flecha arriba / abajo      = eje Z
        // Page Up / Page Down        = eje Y
        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

        if(
            IsKeyDown(
                KEY_LEFT
            )
        )
        {
            posicionModelo.x -=
                velocidadMovimiento *
                dt;
        }


        if(
            IsKeyDown(
                KEY_RIGHT
            )
        )
        {
            posicionModelo.x +=
                velocidadMovimiento *
                dt;
        }


        if(
            IsKeyDown(
                KEY_UP
            )
        )
        {
            posicionModelo.z -=
                velocidadMovimiento *
                dt;
        }


        if(
            IsKeyDown(
                KEY_DOWN
            )
        )
        {
            posicionModelo.z +=
                velocidadMovimiento *
                dt;
        }


        if(
            IsKeyDown(
                KEY_PAGE_UP
            )
        )
        {
            posicionModelo.y +=
                velocidadMovimiento *
                dt;
        }


        if(
            IsKeyDown(
                KEY_PAGE_DOWN
            )
        )
        {
            posicionModelo.y -=
                velocidadMovimiento *
                dt;
        }


        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
        // GIRAR MODELO
        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

        if(
            IsKeyDown(
                KEY_Q
            )
        )
        {
            giroModeloY -=
                velocidadRotacion *
                dt;
        }


        if(
            IsKeyDown(
                KEY_E
            )
        )
        {
            giroModeloY +=
                velocidadRotacion *
                dt;
        }


        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
        // CONTROL DEL SERVO Y DEL CARRITO
        //
        // MODO ESP32:
        // - anguloMotor viene de angulo_orden + desfase visual de 7 grados.
        // - posicionCartX viene de pos_cm real del VL53L0X.
        //
        // MODO MANUAL:
        // - A / D mueven el mecanismo para pruebas sin ESP32.
        // - el carrito se mueve de forma artificial junto con A / D.
        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

        if(modoESP32)
        {
            if(esp32.tieneAngulo)
            {
                float anguloOrden =
                    LimitarFloat(
                        esp32.anguloOrdenGrados,
                        0.0f,
                        180.0f
                    );


                float anguloObjetivoVisual =
                    anguloOrden +
                    DESFASE_SERVO_VISUAL;


                // Suavizado visual. NO altera el valor de la ESP32.
                // Durante RUN, ang llega en la propia telemetria a 5 Hz.

                float factorServo =
                    1.0f -
                    expf(-12.0f * dt);


                anguloMotor +=
                    (anguloObjetivoVisual - anguloMotor) *
                    factorServo;
            }


            if(esp32.tienePosicion)
            {
                float posicionCm =
                    LimitarFloat(
                        esp32.posicionCm,
                        POSICION_MIN_CM,
                        POSICION_MAX_CM
                    );


                float porcentaje =
                    (posicionCm - POSICION_MIN_CM) /
                    (POSICION_MAX_CM - POSICION_MIN_CM);


                float posicionObjetivoX =
                    CART_X_MIN +
                    (CART_X_MAX - CART_X_MIN) *
                    porcentaje;


                float factorCart =
                    1.0f -
                    expf(-10.0f * dt);


                posicionCartX +=
                    (posicionObjetivoX - posicionCartX) *
                    factorCart;
            }
        }
        else
        {
            if(IsKeyDown(KEY_A))
            {
                anguloMotor -=
                    velocidadMotor *
                    dt;
            }


            if(IsKeyDown(KEY_D))
            {
                anguloMotor +=
                    velocidadMotor *
                    dt;
            }


            anguloMotor =
                LimitarFloat(
                    anguloMotor,
                    0.0f,
                    180.0f
                );


            float porcentajeManual =
                anguloMotor /
                180.0f;


            posicionCartX =
                CART_X_MIN +
                (CART_X_MAX - CART_X_MIN) *
                porcentajeManual;
        }


        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
        // ROTACION AUTOMATICA
        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

        if(
            IsKeyPressed(
                KEY_SPACE
            )
        )
        {
            autoRotar =
                !autoRotar;
        }


        if(
            autoRotar
        )
        {
            giroModeloY +=
                18.0f *
                dt;
        }


        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
        // MOSTRAR / OCULTAR PANEL
        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

        if(
            IsKeyPressed(
                KEY_TAB
            )
        )
        {
            mostrarPanel =
                !mostrarPanel;
        }


        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
        // RESET
        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

        if(
            IsKeyPressed(
                KEY_R
            )
        )
        {
            posicionModelo =
                (Vector3)
                {
                    0.0f,
                    0.02f,
                    0.0f
                };


            giroModeloY =
                0.0f;


            escala =
                escalaInicial;


            yaw =
                0.70f;


            pitch =
                0.32f;


            distanciaCamara =
                8.8f;


            autoRotar =
                false;


            anguloMotor =
                90.0f;


            posicionCartX =
                (CART_X_MIN + CART_X_MAX) * 0.5f;
        }


        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
        // CALCULAR POSICION DEL MECANISMO
        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

        Vector2 puntoCrank =
        {
            0.2143266201f,
            0.0732380673f
        };


        Vector2 puntoBeam =
        {
            0.2045000046f,
            0.1626999974f
        };


        float anguloCoupler =
            ANGULO_COUPLER_INICIAL;


        float anguloBeam =
            ANGULO_BEAM_INICIAL;


        bool mecanismoValido =
            CalcularMecanismo(
                anguloMotor,
                &puntoCrank,
                &puntoBeam,
                &anguloCoupler,
                &anguloBeam
            );


        float deltaCoupler =
            anguloCoupler -
            ANGULO_COUPLER_INICIAL;


        float deltaBeam =
            anguloBeam -
            ANGULO_BEAM_INICIAL;


        // El modelo GLB del crank fue exportado con un angulo inicial
        // de ANGULO_CRANK_INICIAL. Para colocarlo en la referencia
        // deseada (0 arriba, 90 derecha, 180 abajo) calculamos
        // solamente la rotacion adicional necesaria.

        float anguloCrankVisual =
            90.0f -
            anguloMotor;


        float deltaCrank =
            anguloCrankVisual -
            ANGULO_CRANK_INICIAL;


        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
        // ACTUALIZAR CAMARA
        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

        // La camara NO sigue la traslacion del ensamble.
        //
        // Antes objetivoCamara usaba posicionModelo.x/y/z, por lo que
        // al mover el modelo con las flechas la camara se movia exactamente
        // junto con el y visualmente parecia que X/Z no hacian nada.
        //
        // Ahora la camara permanece orbitando alrededor de un punto fijo
        // y posicionModelo mueve realmente el ensamble dentro de la escena.

        objetivoCamara =
            (Vector3)
            {
                0.0f,
                1.20f,
                0.0f
            };


        camera.target =
            objetivoCamara;


        camera.position.x =
            objetivoCamara.x +

            distanciaCamara *
            cosf(pitch) *
            sinf(yaw);


        camera.position.y =
            objetivoCamara.y +

            distanciaCamara *
            sinf(pitch);


        camera.position.z =
            objetivoCamara.z +

            distanciaCamara *
            cosf(pitch) *
            cosf(yaw);


        Vector3 cameraPosition =
            camera.position;


        SetShaderValue(
            shader,

            shader.locs[
                SHADER_LOC_VECTOR_VIEW
            ],

            &cameraPosition,

            SHADER_UNIFORM_VEC3
        );


        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
        // DIBUJAR
        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

        BeginDrawing();


        // LIMPIAR PANTALLA Y PROFUNDIDAD (arregla la "nube blanca").
        //
        // BeginDrawing() NO limpia nada. Sin este ClearBackground,
        // el buffer de profundidad conserva lo que se dibujo en el
        // cuadro anterior y tapa partes del modelo cuando la camara
        // se mueve. El color da igual porque el degradado lo cubre.

        ClearBackground(
            FONDO_SUPERIOR
        );


        // Fondo tipo estudio.

        DrawRectangleGradientV(
            0,
            0,

            GetScreenWidth(),
            GetScreenHeight(),

            FONDO_SUPERIOR,
            FONDO_INFERIOR
        );


        BeginMode3D(
            camera
        );


        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
        // PISO
        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

        DrawPlane(
            (Vector3)
            {
                0.0f,
                0.0f,
                0.0f
            },

            (Vector2)
            {
                24.0f,
                24.0f
            },

            PISO_COLOR
        );


        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
        // SOMBRA SUAVE SIMULADA
        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

        DrawCylinder(
            (Vector3)
            {
                posicionModelo.x,
                0.008f,
                posicionModelo.z
            },

            1.75f,
            1.75f,
            0.008f,

            64,

            Fade(
                BLACK,
                0.045f
            )
        );


        DrawCylinder(
            (Vector3)
            {
                posicionModelo.x,
                0.010f,
                posicionModelo.z
            },

            1.25f,
            1.25f,
            0.008f,

            64,

            Fade(
                BLACK,
                0.065f
            )
        );


        DrawCylinder(
            (Vector3)
            {
                posicionModelo.x,
                0.012f,
                posicionModelo.z
            },

            0.80f,
            0.80f,
            0.008f,

            64,

            Fade(
                BLACK,
                0.080f
            )
        );


        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
        // MODELO
        //
        // Se usan las piezas exportadas directamente desde SolidWorks.
        // El shader, materiales, fondo, piso y luces siguen siendo los de Claude.
        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

        // BASE

        DibujarPieza(
            modeloBase,
            (Vector3){0.0f, 0.0f, 0.0f},
            QuaternionIdentity(),
            (Vector3){1.0f, 1.0f, 1.0f},
            escala,
            giroModeloY,
            posicionModelo
        );


        // PIEZAS FIJAS DEL NUEVO ENSAMBLE

        DibujarPieza(
            modeloHingeTower,
            (Vector3){-0.2475000024f, 0.0126999998f, 0.0063999998f},
            QuaternionIdentity(),
            (Vector3){1.0f, 1.0f, 1.0f},
            escala,
            giroModeloY,
            posicionModelo
        );


        DibujarPieza(
            modeloBaseServo,
            (Vector3){0.1474999934f, 0.0126999998f, -0.0177999996f},
            QuaternionIdentity(),
            (Vector3){1.0f, 1.0f, 1.0f},
            escala,
            giroModeloY,
            posicionModelo
        );


        DibujarPieza(
            modeloServo,
            (Vector3){0.1733600050f, 0.0617999993f, -0.0556499995f},
            (Quaternion){0.7071068287f, 0.0f, 0.0f, 0.7071068287f},
            (Vector3){1.0f, 1.0f, 1.0f},
            escala,
            giroModeloY,
            posicionModelo
        );


        if(
            mecanismoValido
        )
        {
            // CRANK DEL SERVO

            Quaternion qCrank =
                RotacionExtraZ(
                    (Quaternion){0.0f, 0.0f, 0.0232008789f, 0.9997308254f},
                    deltaCrank
                );


            DibujarPieza(
                modeloCrank,
                (Vector3){0.1833599955f, 0.0718000010f, -0.0163000003f},
                qCrank,
                (Vector3){1.0f, 1.0f, 1.0f},
                escala,
                giroModeloY,
                posicionModelo
            );


            // RODAMIENTOS DEL EXTREMO DEL CRANK

            Vector3 bearing1Pos =
                RotarPuntoZ(
                    (Vector3){0.2143266201f, 0.0732380673f, 0.0066999998f},
                    (Vector2){PIVOTE_MOTOR_X, PIVOTE_MOTOR_Y},
                    deltaCrank
                );


            Vector3 bearing4Pos =
                RotarPuntoZ(
                    (Vector3){0.2143266201f, 0.0732380673f, 0.0017000000f},
                    (Vector2){PIVOTE_MOTOR_X, PIVOTE_MOTOR_Y},
                    deltaCrank
                );


            DibujarPieza(
                modeloBearing,
                bearing1Pos,
                RotacionExtraZ(
                    (Quaternion){0.0f, 0.0f, -0.1586437672f, 0.9873359203f},
                    deltaCrank
                ),
                (Vector3){1.0f, 1.0f, 1.0f},
                escala,
                giroModeloY,
                posicionModelo
            );


            DibujarPieza(
                modeloBearing,
                bearing4Pos,
                RotacionExtraZ(
                    (Quaternion){0.0f, 0.0f, 0.0270696115f, 0.9996335506f},
                    deltaCrank
                ),
                (Vector3){1.0f, 1.0f, 1.0f},
                escala,
                giroModeloY,
                posicionModelo
            );


            // BIELA / COUPLER

            Vector3 posicionCoupler =
            {
                (
                    puntoCrank.x +
                    puntoBeam.x
                ) *
                0.5f,

                (
                    puntoCrank.y +
                    puntoBeam.y
                ) *
                0.5f,

                0.0066999998f
            };


            DibujarPieza(
                modeloCoupler,
                posicionCoupler,
                RotacionExtraZ(
                    (Quaternion){0.0f, 0.0f, -0.6673886180f, 0.7447096109f},
                    deltaCoupler
                ),
                (Vector3){1.0f, 1.0f, 1.0f},
                escala,
                giroModeloY,
                posicionModelo
            );


            // TODO EL BEAM GIRA ALREDEDOR DEL PIVOTE IZQUIERDO

            Vector2 pivoteBeam =
            {
                PIVOTE_BEAM_X,
                PIVOTE_BEAM_Y
            };


            Vector3 dowel2Pos =
                RotarPuntoZ(
                    (Vector3){-0.2134999931f, 0.1626999974f, -0.0048000002f},
                    pivoteBeam,
                    deltaBeam
                );


            Vector3 dowel1Pos =
                RotarPuntoZ(
                    (Vector3){-0.2134999931f, 0.1626999974f, 0.0172000006f},
                    pivoteBeam,
                    deltaBeam
                );


            DibujarPieza(
                modeloDowel,
                dowel2Pos,
                RotacionExtraZ(
                    (Quaternion){0.3519707918f, 0.6132834554f, 0.3519707918f, 0.6132834554f},
                    deltaBeam
                ),
                (Vector3){1.0f, 1.0f, 1.0f},
                escala,
                giroModeloY,
                posicionModelo
            );


            DibujarPieza(
                modeloDowel,
                dowel1Pos,
                RotacionExtraZ(
                    (Quaternion){-0.2309943289f, 0.6683125496f, -0.2309943289f, 0.6683124900f},
                    deltaBeam
                ),
                (Vector3){1.0f, 1.0f, 1.0f},
                escala,
                giroModeloY,
                posicionModelo
            );


            Vector3 beamEndPos =
                RotarPuntoZ(
                    (Vector3){0.1835000068f, 0.1626999974f, 0.0062000002f},
                    pivoteBeam,
                    deltaBeam
                );


            DibujarPieza(
                modeloBeamEnd,
                beamEndPos,
                RotacionExtraZ(
                    (Quaternion){0.0f, 0.7071068287f, 0.0f, 0.7071068287f},
                    deltaBeam
                ),
                (Vector3){1.0f, 1.0f, 1.0f},
                escala,
                giroModeloY,
                posicionModelo
            );


            Vector3 beamHingePos =
                RotarPuntoZ(
                    (Vector3){-0.2105000019f, 0.1626999974f, 0.0062000002f},
                    pivoteBeam,
                    deltaBeam
                );


            DibujarPieza(
                modeloBeamHinge,
                beamHingePos,
                RotacionExtraZ(
                    (Quaternion){0.0f, -0.7071068287f, 0.0f, 0.7071068287f},
                    deltaBeam
                ),
                (Vector3){1.0f, 1.0f, 1.0f},
                escala,
                giroModeloY,
                posicionModelo
            );


            // SENSOR GYUL53L0X SOBRE EL BEAM

            Vector3 sensorPos =
                RotarPuntoZ(
                    (Vector3){-0.2125000060f, 0.1834699959f, 0.0078450004f},
                    pivoteBeam,
                    deltaBeam
                );


            DibujarPieza(
                modeloSensor,
                sensorPos,
                RotacionExtraZ(
                    (Quaternion){0.0f, 0.7071068287f, 0.0f, 0.7071068287f},
                    deltaBeam
                ),
                (Vector3){1.0f, 1.0f, 1.0f},
                escala,
                giroModeloY,
                posicionModelo
            );


            // REGLA ADHERIDA AL BEAM

            Vector3 reglaPos =
                RotarPuntoZ(
                    (Vector3){-0.2006999999f, 0.1329749972f, 0.0312350001f},
                    pivoteBeam,
                    deltaBeam
                );


            DibujarPieza(
                modeloRegla,
                reglaPos,
                RotacionExtraZ(
                    QuaternionIdentity(),
                    deltaBeam
                ),
                (Vector3){1.0f, 1.0f, 1.0f},
                escala,
                giroModeloY,
                posicionModelo
            );


            // CARRO

            Vector3 cartPos =
                RotarPuntoZ(
                    (Vector3)
                    {
                        posicionCartX,
                        CART_Y_EJE,
                        CART_Z_EJE
                    },

                    pivoteBeam,
                    deltaBeam
                );


            DibujarPieza(
                modeloCart,
                cartPos,
                RotacionExtraZ(
                    QuaternionIdentity(),
                    deltaBeam
                ),
                (Vector3){1.0f, 1.0f, 1.0f},
                escala,
                giroModeloY,
                posicionModelo
            );


            // RODAMIENTOS SUPERIORES DEL BEAM

            Vector3 bearing3Pos =
                RotarPuntoZ(
                    (Vector3){0.2045000046f, 0.1626999974f, 0.0066999998f},
                    pivoteBeam,
                    deltaBeam
                );


            Vector3 bearing2Pos =
                RotarPuntoZ(
                    (Vector3){0.2045000046f, 0.1626999974f, 0.0017000000f},
                    pivoteBeam,
                    deltaBeam
                );


            DibujarPieza(
                modeloBearing,
                bearing3Pos,
                RotacionExtraZ(
                    (Quaternion){0.0f, 0.0f, 0.6201135516f, 0.7845120430f},
                    deltaBeam
                ),
                (Vector3){1.0f, 1.0f, 1.0f},
                escala,
                giroModeloY,
                posicionModelo
            );


            DibujarPieza(
                modeloBearing,
                bearing2Pos,
                RotacionExtraZ(
                    (Quaternion){0.0f, 0.0f, 0.0913516730f, 0.9958187938f},
                    deltaBeam
                ),
                (Vector3){1.0f, 1.0f, 1.0f},
                escala,
                giroModeloY,
                posicionModelo
            );
        }


        EndMode3D();


        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
        // TITULO
        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

        DrawText(
            "BALL AND BEAM - GEMELO DIGITAL",
            30,
            25,
            25,
            (Color){30,36,38,255}
        );


        DrawText(
            autoRotar ?
            "AUTO ROTACION: ON" :
            "AUTO ROTACION: OFF",

            30,
            60,
            17,

            autoRotar ?
            VERDE :
            DARKGRAY
        );


        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
        // PANEL
        // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

        if(
            mostrarPanel
        )
        {
            DrawRectangleRounded(
                (Rectangle)
                {
                    25,
                    105,
                    500,
                    710
                },

                0.06f,
                10,

                PANEL_COLOR
            );


            DrawText(
                "VISUALIZADOR 3D",
                50,
                130,
                24,
                WHITE
            );


            DrawText(
                "Estado del gemelo",
                50,
                165,
                18,
                VERDE
            );


            DrawLine(
                50,
                198,
                425,
                198,

                Fade(WHITE, 0.20f)
            );


            DrawText(
                modoESP32 ? "Modo: ESP32" : "Modo: MANUAL",
                50,
                218,
                18,
                modoESP32 ? VERDE : ORANGE
            );


            DrawText(
                esp32.conectado ? "ESP32: CONECTADA" : "ESP32: SIN CONEXION",
                50,
                245,
                18,
                esp32.conectado ? VERDE : RED
            );


            DrawText(
                TextFormat("Puerto: %s @ 115200", PUERTO_ESP32),
                50,
                272,
                17,
                TEXTO_CLARO
            );


            DrawText(
                esp32.appReady ?
                "APP: READY v1" :
                "APP: ESPERANDO HANDSHAKE",
                50,
                299,
                17,
                esp32.appReady ? VERDE : ORANGE
            );


            DrawText(
                esp32.controlArmado ?
                "Control: ARMADO" :
                "Control: PAUSADO",
                50,
                326,
                17,
                esp32.controlArmado ? VERDE : ORANGE
            );


            DrawText(
                esp32.latidoActivo ?
                "Latido: PING 250 ms" :
                "Latido: detenido",
                50,
                353,
                17,
                esp32.latidoActivo ? VERDE : LIGHTGRAY
            );


            DrawText(
                TextFormat(
                    "Servo ESP32: %.2f deg",
                    esp32.anguloOrdenGrados
                ),
                50,
                387,
                18,
                TEXTO_CLARO
            );


            DrawText(
                TextFormat(
                    "Servo visual: %.2f deg",
                    anguloMotor
                ),
                50,
                414,
                18,
                TEXTO_CLARO
            );


            DrawText(
                TextFormat(
                    "Posicion real: %.2f cm",
                    esp32.posicionCm
                ),
                50,
                441,
                18,
                TEXTO_CLARO
            );


            DrawText(
                TextFormat(
                    "Referencia: %.1f cm",
                    esp32.referenciaCm
                ),
                50,
                468,
                18,
                TEXTO_CLARO
            );


            DrawText(
                TextFormat(
                    "Estado: %s",
                    esp32.estado
                ),
                50,
                495,
                18,
                TEXTO_CLARO
            );


            DrawText(
                TextFormat(
                    "Carrito 3D X: %.1f mm",
                    posicionCartX * 1000.0f
                ),
                50,
                522,
                18,
                TEXTO_CLARO
            );


            DrawText(
                TextFormat(
                    "ESP32: %.54s",
                    esp32.ultimoMensaje
                ),
                50,
                549,
                15,
                LIGHTGRAY
            );


            DrawLine(
                50,
                578,
                475,
                578,
                Fade(WHITE, 0.20f)
            );


            DrawText(
                "CONTROLES",
                50,
                594,
                18,
                VERDE
            );


            DrawText(
                "Mouse izq. Orbitar   Rueda Zoom",
                50,
                621,
                15,
                WHITE
            );


            DrawText(
                "Flechas: mover X/Z   Q/E: girar",
                50,
                644,
                15,
                WHITE
            );


            DrawText(
                "M: ESP32/manual   C: reconectar",
                50,
                667,
                15,
                WHITE
            );


            DrawText(
                "F5: RUN   F6: STOP   F7: STATUS",
                50,
                690,
                15,
                WHITE
            );
        }


        DrawText(
            "F5 RUN  F6 STOP  F7 STATUS  M modo  C reconectar  TAB panel",

            GetScreenWidth() -
            475,

            GetScreenHeight() -
            35,

            16,

            (Color)
            {
                45,
                52,
                55,
                255
            }
        );


        DrawFPS(
            GetScreenWidth() -
            90,

            20
        );


        EndDrawing();
    }


    // Cerrar el puerto antes de liberar raylib.
    CerrarESP32(&esp32);


    // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~
    // LIBERAR RECURSOS
    // ~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~<3~

    UnloadShader(
        shader
    );


    UnloadModel(modeloBase);
    UnloadModel(modeloHingeTower);
    UnloadModel(modeloServo);
    UnloadModel(modeloBaseServo);
    UnloadModel(modeloCrank);
    UnloadModel(modeloCoupler);
    UnloadModel(modeloDowel);
    UnloadModel(modeloBeamEnd);
    UnloadModel(modeloBeamHinge);
    UnloadModel(modeloBearing);
    UnloadModel(modeloSensor);
    UnloadModel(modeloCart);
    UnloadModel(modeloRegla);


    CloseWindow();


    return 0;
}