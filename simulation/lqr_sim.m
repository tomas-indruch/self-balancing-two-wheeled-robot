clc; clear; close all;

%% 1. Fyzikální parametry robota
g  = 9.81;            % Gravitace [m/s^2]
init_tilt = 15;       % Stupně

% --- Parametry Těla (Šasi) ---
body_mass = 0.8;      % Hmotnost těla [kg] (mb)
body_width = 0.10;    % Šířka těla [m]
body_depth = 0.04;    % Hloubka (tloušťka) těla [m]
body_height = 0.12;   % Výška těla [m]

% Kde je osa kol? (měřeno od spodní hrany těla nahoru)
axle_z_pos = 0.015;   

% Výpočet L (Vzdálenost mezi osou kol a Těžištěm těla)
% Předpokládáme těžiště v polovině výšky těla
center_of_mass_height = body_height / 2;
L = center_of_mass_height - axle_z_pos; 

% --- Parametry Kol ---
wheel_mass = 0.058;   % Hmotnost jednoho kola [kg] (mw)
r = 0.035;            % Poloměr kola [m] (70mm průměr)
wheel_width = 0.02;   % Šířka kola [m]

% --- Momenty setrvačnosti ---
% 1. Setrvačnost těla kolem jeho VLASTNÍHO těžiště (CoM)
% Vzorec pro kvádr: I = 1/12 * m * (hloubka^2 + vyska^2)
Ib = (1/12) * body_mass * (body_depth^2 + body_height^2);

% 2. Setrvačnost kola (Válec)
Iw = 0.5 * wheel_mass * r^2;

% --- Tření ---
bx = 0.01;       % Odpor pohybu vozíku [Ns/m]
btheta = 0.001;  % Odpor v kyvu (ložiska) [Nms/rad]

%% 2. Pomocné substituce (Dynamické koeficienty)
% mb = body_mass, mw = wheel_mass
mb = body_mass; 
mw = wheel_mass;

% alpha = Efektivní hmotnost vozíku (zahrnuje rotační energii kol)
alpha = (2 * mw) + mb + (2 * Iw / r^2);

% beta = Vazební člen (coupling) mezi posuvem a kyvem
beta = mb * L;

% gamma = Efektivní setrvačnost kyvadla (Steinerova věta: I_com + m*L^2)
gamma = Ib + (mb * L^2);

% Determinant soustavy
Det = (alpha * gamma) - (beta^2);

%% 3. Sestavení matic A a B (Linearizovaný model)
% Nový vektor stavů: x = [pozice; rychlost; uhel; rychlost_uhlu]

A = [0, 1,                                       0,                                   0;
     0, (-gamma * bx)/Det,                       (-beta * mb * g * L)/Det,            (beta * btheta)/Det;
     0, 0,                                       0,                                   1;
     0, (beta * bx)/Det,                         (alpha * mb * g * L)/Det,            (-alpha * btheta)/Det];

B = [0;
     ((gamma / r) + beta) / Det;
     0;
     ((-beta / r) - alpha) / Det];

C = eye(4);
D = [0;0;0;0];

%% 4. Návrh DISKRÉTNÍHO LQR regulátoru pro STM32
% Vzorkovací frekvence 100 Hz
Ts = 0.01; 

% Penále za chybu stavu [Pozice, Rychlost, Úhel, Rychlost_úhlu]
% Pozici dáme penalizaci např. 20, ať se na ni snaží vrátit
Q = diag([5, 0, 5000, 50]); 
R = 0.05;

% Správný výpočet pro převod analogového návrhu do mikrokontroléru
K_d = lqrd(A, B, Q, R, Ts);

disp('Spojitý zisk K (ideální fyzika):');
disp(lqr(A, B, Q, R));
disp('Diskrétní zisk K_d (pro STM32):');
disp(K_d);