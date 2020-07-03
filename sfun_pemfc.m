function sfun_pemfc(block)
% Plantilla para implementar modelos en variable de estado sencillos
% como una S-Function
% 
% El nombre del archivo y el de la función (ver linea 1) deben ser iguales

%
% Acá solo se llama a la función setup, no agregue nada más a esta función
% principal
%
setup(block);

% -------------------------------------------------------------------------
% Function: setup ===================================================
% Abstract:
% Acá se definen las caracterísiticas básicas del bloque de la S-function:
%   - Puertos de entrada
%   - Puertos de salida
%   - Definición de parámetros
%   - Opciones
%
%
function setup(block)

%De las ecuaciones 19 y 20 se tiene que:

% x = [pH_2 pO_2 pH_2*O_C]^T %Presiones parciales de cada gas
% u = [H_2in O_2in i]^T  %i es la intensidad de corriente
% y = V

% R = 8.3144 % Constantes de gas ideal
% F = 96439 % Constante de Faraday
% P_std = 101325 % Presión estándar

% Se registra el número de puertos de entrada y salida
block.NumInputPorts  = 2; % Cantidad de entradas
block.NumOutputPorts = 1; % Cantidad de salidas
% OJO ====>>> Inicialmente el sistema contempla una sola salida pero luego
              % de la linealización son 3 salidas

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Propiedades de los puertos de entrada
block.InputPort(1).Dimensions        = 1;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = true;

% Propiedades de los puertos de entrada
block.InputPort(2).Dimensions        = 1;
block.InputPort(2).DatatypeID  = 0;  % double
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).DirectFeedthrough = true;

% Propiedades de los puertos de salida
block.OutputPort(1).Dimensions       = 1;
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';
block.OutputPort(1).SamplingMode = 'Sample';

block.OutputPort(2).Dimensions       = 1;
block.OutputPort(2).DatatypeID  = 0; % double
block.OutputPort(2).Complexity  = 'Real';
block.OutputPort(2).SamplingMode = 'Sample';

% Número de parámetros
block.NumDialogPrms     = 9; % en este caso hay oho parámetros de entrada
                             % el noveno "parámentro" corresponde al vector
                             % de entrada

block.SampleTimes = [0, 0]; % Tiempo de muestreo heredado

block.NumContStates = 3; % Definicion de la cantidad de variables de estado
% Inicialmente son 3, pero luego de linealizarlo son 5 OJO <====
  
block.SimStateCompliance = 'DefaultSimState';
% -----------------------------------------------------------------
% Ahora se registran los métodos internos de la S-function
% -----------------------------------------------------------------
block.RegBlockMethod('InitializeConditions', @Inicializacion);
block.RegBlockMethod('Outputs', @Salidas);     
block.RegBlockMethod('Derivatives', @ModeloEstados);
block.RegBlockMethod('SetInputPortSamplingMode',@SetInputPortSamplingMode); % Necesario para tener dos salidas
%end setup

function Inicializacion(block)

block.ContStates.Data = block.DialogPrm(13).Data; % esto lo que hace es
%             tomar el valor del estado inicial como si fuera un parámetro
%end Inicializacion


function Salidas(block)
% Acá se escribe las ecuaciones de salida
% y = V <= Sin aplicar linealización exacta, V es igual a la ecuación 20

N  = block.DialogPrm(4).Data; % Número de celdas
Eo = block.DialogPrm(7).Data; % Tensión sin carga
T  = block.DialogPrm(6).Data; % Temperatura de operación

R = 8.3144 % Constantes de gas ideal
F = 96439 % Constante de Faraday
P_std = 101325 % Presión estándar

% x = [pH_2 pO_2 pH_2*O_C]^T %Presiones parciales de cada gases
x = block.ContStates.Data; % el estado actual
pH_2 = x(1);
pO_2 = x(2);
pH_2O = x(3);

% u = [H_2in O_2in i]^T
O = block.InputPort(2).Data; % Solamente se requiere la cantidad de oxígeno

%Se requiere para deternimar L
% i, i_n, i_o, i_l, r, a y b                                    <==========
L = 0; %Aún no se conocen los parámetros necesarios
%Determinar de donde salen todas estas corrientes               <==========
% Esta condición en Cero crearía un sistema sin pérdidas

V_1 = N*(Eo+((R*T)/(2*F))*ln((pH_2*sqrt(pO_2/P_std))/pH_2O)-L);

block.OutputPort(1).Data = V_1; % Salida 1
%Dos salidas extra del modelo linealizado
%block.OutputPort(2).Data = x4 o V_2 % Salida 2
%block.OutputPort(2).Data = x5 o V_3; % Salida 3

%end Salidas

function ModeloEstados(block)
% Acá se escribe la función que calcula las derivadas de las variables de
% estado
Ac = block.DialogPrm(1).Data; % Área activa
Va = block.DialogPrm(2).Data; % Volumen del anodo
Vc = block.DialogPrm(3).Data; % Volumen del cátodo
N  = block.DialogPrm(4).Data; % Número de celdas
Po = block.DialogPrm(5).Data; % Presion de operación
T  = block.DialogPrm(6).Data; % Temperatura de operación
Eo = block.DialogPrm(7).Data; % Tensión sin carga
Fu = block.DialogPrm(8).Data; % Factor de utilización
Ref = block.DialogPrm(9).Data; % Constante de reformador
Fc = block.DialogPrm(10).Data; % Factor de conversión
Met_sig = block.DialogPrm(11).Data; % Referencia de señal de metano 0.000015
HO_fr = block.DialogPrm(12).Data; % Radio de flujo Hidrógeno-Oxígeno

% x = [pH_2 pO_2 pH_2O_C]^T %Presiones parciales de cada gas
x = block.ContStates.Data; % Vector de estados iniciales
pH_2 = x(1);
pO_2 = x(2);
pH_2O_C = x(3);

% u = [H_2in O_2in i]^T
H_2in =  block.InputPort(1).Data; % Hidrógeno de entrada
O_2in =  block.InputPort(2).Data; % Oxígeno de entrada
i =  block.InputPort(2).Data; % i es la intensidad de corriente

%=====Desarrollar las matrices de estado aquí=====
%=================================================


block.Derivatives.Data = [dx1dt;dx2dt;dx3dt]; % actualizacion del bloque de la S-function
%end ModeloEstados

function SetInputPortSamplingMode(s, port, mode)
s.InputPort(port).SamplingMode = mode;
%end SetInputPortSamplingMode