% Implementación para Matlab
% Modelo Linealizado de una celda de combustible mediante linealización exacta
% Ricardo Mena Cortés
% Universidad de Costa Rica
% Modelo basado en el artículo:
% Nonlinear Control of PEM Fuel Cells by Exact Linearization
% Autores: Woon Ki Na, Bei Gou, and Bill Diong

function sfun_pemfc_lineal(block)
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
block.NumInputPorts  = 4; % Cantidad de entradas
block.NumOutputPorts = 4; % Cantidad de salidas

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

% Propiedades de los puertos de entrada
block.InputPort(4).Dimensions        = 1;
block.InputPort(4).DatatypeID  = 0;  % double
block.InputPort(4).Complexity  = 'Real';
block.InputPort(4).DirectFeedthrough = true;

% Propiedades de los puertos de salida
block.OutputPort(1).Dimensions       = 1;
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';
block.OutputPort(1).SamplingMode = 'Sample';

% Propiedades de los puertos de salida
block.OutputPort(2).Dimensions       = 1;
block.OutputPort(2).DatatypeID  = 0; % double
block.OutputPort(2).Complexity  = 'Real';
block.OutputPort(2).SamplingMode = 'Sample';

% Propiedades de los puertos de salida
block.OutputPort(3).Dimensions       = 1;
block.OutputPort(3).DatatypeID  = 0; % double
block.OutputPort(3).Complexity  = 'Real';
block.OutputPort(3).SamplingMode = 'Sample';

% Propiedades de los puertos de salida
block.OutputPort(4).Dimensions       = 3;
block.OutputPort(4).DatatypeID  = 0; % double
block.OutputPort(4).Complexity  = 'Real';
block.OutputPort(4).SamplingMode = 'Sample';

% Número de parámetros
block.NumDialogPrms     = 13; % en este caso hay 12 parámetros de entrada
                             % el 13 "parámentro" corresponde al vector
                             % de entrada

block.SampleTimes = [0, 0]; % Tiempo de muestreo heredado

block.NumContStates = 5; % Definicion de la cantidad de variables de estado

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
N  = 35; % Número de celdas
T  = 338.5; % Temperatura de operación
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

%L = (3.0762e-9);

% Tensión de Salida
V = N*(Eo+((R*T)/(2*F))*log((pH_2*sqrt(pO_2/P_std))/pH_2O)-L);

% Corriente de salida
%I = Ac*i;

block.OutputPort(1).Data = V; % Salida 1
block.OutputPort(2).Data = x(4); % Salida 1
block.OutputPort(3).Data = x(5); % Salida 1
block.OutputPort(4).Data = [x(1); x(2); x(3)]; % Salida 1
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
H_2O_in = block.InputPort(4).Data; % Agua de entrada

R = 8.3144; % Constantes de gas ideal
F = 96439; % Constante de Faraday

K_r = N/(4*F); %

RTA = (R*T)/Va;
RTC = (R*T)/Vc;

M_1_1 = RTA*(1-(x(1)/Po));
M_2_2 = RTC*(1-(x(2)/Po));
M_2_3 = -RTC*(x(3)/Po);

M_3_1 = RTA*(2*K_r*Ac*(-1+(x(1)/Po)));
M_3_2 = RTC*(K_r*Ac*(-1+(x(2)/Po)));
M_3_3 = RTC*(2*K_r*Ac*(1-(x(3)/Po)));

%=====Desarrollar las matrices de estado aquí=====
dx1dt = M_1_1*H_2in+M_3_1*i;
dx2dt = M_2_2*O_2in+M_3_2*i;
dx3dt = RTC*H_2O_in+M_2_3*O_2in+M_3_3*i;%RTC*H_2O_in+
dx4dt = O_2in;
dx5dt = i;
%=================================================

block.Derivatives.Data = [dx1dt;dx2dt;dx3dt;dx4dt;dx5dt]; % actualizacion del bloque de la S-function
%end ModeloEstados

function SetInputPortSamplingMode(s, port, mode)
s.InputPort(port).SamplingMode = mode;
%end SetInputPortSamplingMode