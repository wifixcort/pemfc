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

% Se registra el número de puertos de entrada y salida
block.NumInputPorts  = 2; % Cantidad de entradas
block.NumOutputPorts = 2; % Cantidad de salidas

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

block.NumContStates = 2; % Definicion de la cantidad de variables de estado
  
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

block.ContStates.Data = block.DialogPrm(9).Data; % esto lo que hace es
%             tomar el valor del estado inicial como si fuera un parámetro
%end Inicializacion

%
function Salidas(block)
% Acá se escribe las ecuaciones de salida
x = block.ContStates.Data; % el estado actual
block.OutputPort(1).Data = 1; % En este caso, la salida 1
block.OutputPort(2).Data = 1; % En este caso, la salida 2
%end Salidas

function ModeloEstados(block)
% Acá se escribe la función que calcula las derivadas de las variables de
% estado
Ac = block.DialogPrm(1).Data; % Área activa
Va = block.DialogPrm(2).Data; % Volumen del anodo
Vc = block.DialogPrm(3).Data; % Volumen del cátodo
N  = block.DialogPrm(4).Data; % Número de celdas
Po = block.DialogPrm(5).Data; % Presion de operacion
T  = block.DialogPrm(6).Data; % Temperatura de operación
Eo = block.DialogPrm(7).Data; % Tensión sin carga
Fu = block.DialogPrm(8).Data; % Factor de utilización

x = block.ContStates.Data; % el valor del estado actual
u1 =  block.InputPort(1).Data; % el valor de la entrada 1 actual
u2 =  block.InputPort(2).Data; % el valor de la entrada 1 actual

block.Derivatives.Data = [dx1dt;dx2dt]; % actualizacion del bloque de la S-function
%end ModeloEstados

function SetInputPortSamplingMode(s, port, mode)
s.InputPort(port).SamplingMode = mode;
%end SetInputPortSamplingMode