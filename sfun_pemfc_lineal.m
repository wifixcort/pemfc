% Implementación para Matlab
% Modelo Linealizado de una celda de combustible mediante linealización exacta
% Ricardo Mena Cortés
% Universidad de Costa Rica
% Modelo basado en el artículo:
% Nonlinear Control of PEM Fuel Cells by Exact Linearization
% Autores: Woon Ki Na, Bei Gou, and Bill Diong

function sfun_pemfc(block)
% 
% No agregue nada más a esta función principal
%
setup(block);

% -------------------------------------------------------------------------
% Function: setup ===================================================
% Abstract:
% Se pretende modelar una celda de combustible con el siguiente con las
% siguientes características:
%
% x = [pH_2 pO_2 pH_2*O_C]^T % Presiones parciales de cada gas
% u = [H_2in O_2in i]^T  % Entradas de la celda
% y = V % Tensión de salida de la celda
%
function setup(block)
% Se registra el número de puertos de entrada y salida
block.NumInputPorts  = 3; % Cantidad de entradas
block.NumOutputPorts = 1; % Cantidad de salidas

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

% Propiedades de los puertos de entrada
block.InputPort(3).Dimensions        = 1;
block.InputPort(3).DatatypeID  = 0;  % double
block.InputPort(3).Complexity  = 'Real';
block.InputPort(3).DirectFeedthrough = true;

% Propiedades de los puertos de salida
block.OutputPort(1).Dimensions       = 1;
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';
block.OutputPort(1).SamplingMode = 'Sample';

% Propiedades de los puertos de salida
%block.OutputPort(2).Dimensions       = 1;
%block.OutputPort(2).DatatypeID  = 0; % double
%block.OutputPort(2).Complexity  = 'Real';
%block.OutputPort(2).SamplingMode = 'Sample';

% Número de parámetros
block.NumDialogPrms     = 13; % en este caso hay 12 parámetros de entrada
                             % el 13 "parámentro" corresponde al vector
                             % de entrada

block.SampleTimes = [0, 0]; % Tiempo de muestreo heredado

block.NumContStates = 3; % Definicion de la cantidad de variables de estado

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

Ac = block.DialogPrm(1).Data; % Área activa

N  = block.DialogPrm(4).Data; % Número de celdas
T  = block.DialogPrm(6).Data; % Temperatura de operación
Eo = block.DialogPrm(7).Data; % Tensión sin carga

R = 8.3144; % Constantes de gas ideal
F = 96439; % Constante de Faraday
P_std = 101325; % Presión estándar

% x = [pH_2 pO_2 pH_2*O_C]^T %Presiones parciales de cada gases
x = block.ContStates.Data; % el estado actual
pH_2 = x(1);
pO_2 = x(2);
pH_2O = x(3);

% u = [H_2in O_2in i]^T
i = block.InputPort(3).Data; % Solamente se requiere la cantidad de oxígeno

%Se requiere para deternimar L
% Valores típicos
i_o = 0.1*Ac; % mA cm^-2
i_n = 3; % mA cm^−2
i_l = 1000; % mA cm^-2
a = 0.06; %v
b = 0.05; %v
r = (3.0762e-9)*Ac;%(30.762e-6)u*ohmn m^-2

%L = 0.1; %Aún no se conocen los parámetros necesarios
%Determinar de donde salen todas estas corrientes               <==========
% Esta condición en Cero crearía un sistema sin pérdidas
L = (i+i_n)*r+a*log((i+i_n)/i_o)-b*log(1-((i+i_n)/i_l));

% Tensión de Salida
V = N*(Eo+((R*T)/(2*F))*log((pH_2*sqrt(pO_2/P_std))/pH_2O)-L);

% Corriente de salida
%I = Ac*i;

block.OutputPort(1).Data = V; % Salida 1
%block.OutputPort(2).Data = I; % Salida 1
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

% x = [pH_2 pO_2 pH_2O_C]^T %Presiones parciales de cada gas
x = block.ContStates.Data; % Vector de estados iniciales
%pH_2 = x(1);
%pO_2 = x(2);
%pH_2O_C = x(3);

% u = [H_2in O_2in i]^T
H_2in =  block.InputPort(1).Data; % Hidrógeno de entrada
O_2in =  block.InputPort(2).Data; % Oxígeno de entrada
i =  block.InputPort(3).Data; % i es la intensidad de corriente

R = 8.3144; % Constantes de gas ideal
F = 96439; % Constante de Faraday

K_r = N/(4*F); %

M_1_1 = ((R*T)/Va)*(1-(x(1)/Po));
M_2_2 = ((R*T)/Vc)*(1-(x(2)/Po));
M_2_3 = -((R*T)/(Va*Po))*x(3);

RT = (R*T)/(Vc*Po);

M_3_1 = RT*(-2*K_r*Ac+2*K_r*Ac*x(1));
M_3_2 = RT*(-K_r*Ac+2*K_r*Ac*x(2));
M_3_3 = RT*(2*K_r*Ac-2*K_r*Ac*x(3));

%=====Desarrollar las matrices de estado aquí=====
dx1dt = M_1_1*H_2in+M_3_1*i;
dx2dt = M_2_2*O_2in+M_3_2*i;
dx3dt = M_2_3*O_2in+M_3_3*i;
%=================================================

block.Derivatives.Data = [dx1dt;dx2dt;dx3dt]; % actualizacion del bloque de la S-function
%end ModeloEstados

function SetInputPortSamplingMode(s, port, mode)
s.InputPort(port).SamplingMode = mode;
%end SetInputPortSamplingMode