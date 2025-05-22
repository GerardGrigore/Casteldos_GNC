clear all
clc

% 1. Définir les Matrices du Modèle
A = [1.1269 -0.4940 0.1129;
     1.0000  0.0000 0.0000;
     0.0000  1.0000 0.0000];

B = [-0.3832;
      0.5919;
      0.5191];

C = [1 0 0;
     0 1 0];  % Nouvelle matrice C avec 2 lignes pour 2 sorties

D = 0;

% 2. Créer le Modèle d'État-Espace
Ts = -1;  % Discret sans échantillonnage spécifique
sys = ss(A, [B B], C, D, Ts, 'InputName', {'u', 'w'}, 'OutputName', {'yt1', 'yt2'});

% 3. Définir les Matrices de Bruit de Processus et de Mesure
Q = 2.3;  % Bruit de processus (reste un scalaire)

R = [1 0;   % Bruit de mesure, matrice diagonale 2x2
     0 1];

% 4. Calculer le Filtre de Kalman
[kalmf, L, ~, Mx, Z] = kalman(sys, Q, R);
kalmf = kalmf(1:2, :);  % Conserver les deux sorties

% 5. Créer les Blocs de Somme Correctement
vIn1 = sumblk('y1 = yt1 + v1');
vIn2 = sumblk('y2 = yt2 + v2');

% 6. Définir les Noms des Entrées et Sorties pour kalmf
kalmf.InputName = {'u', 'y1', 'y2'};
kalmf.OutputName = {'ye1', 'ye2'};

% 7. Connecter les Modèles
SimModel = connect(sys, vIn1, vIn2, kalmf, {'u', 'w', 'v1', 'v2'}, {'y1', 'y2', 'ye1', 'ye2'});

% 8. Définir le Signal d'Entrée pour la Simulation
t = (0:100)';  % Vecteur de temps
u = sin(t / 5);  % Signal d'entrée sinusoïdal

% 9. Définir les Bruits de Processus et de Mesure en Simulation
rng(10, 'twister');  % Initialiser le générateur de nombres aléatoires
w = sqrt(Q) * randn(length(t), 1);  % Bruit de processus
v = randn(length(t), 2) * sqrt(R);  % Bruit de mesure avec deux colonnes

% 10. Simuler le Système
out = lsim(SimModel, [u, w, v]);

% 11. Extraire et Visualiser les Résultats
yt1 = out(:, 1);  % Réponse vraie pour la première sortie
yt2 = out(:, 2);  % Réponse vraie pour la deuxième sortie
ye1 = out(:, 3);  % Réponse filtrée pour la première sortie
ye2 = out(:, 4);  % Réponse filtrée pour la deuxième sortie
y1 = yt1 + v(:, 1);  % Réponse mesurée pour la première sortie
y2 = yt2 + v(:, 2);  % Réponse mesurée pour la deuxième sortie

% Créer la figure pour les graphiques
clf;
subplot(211);
plot(t, yt1, 'b', t, ye1, 'r--', t, y1, 'g--');
xlabel('Number of Samples');
ylabel('Output');
title('Kalman Filter Response - Output 1');
legend('True', 'Filtered', 'Measured');

subplot(212);
plot(t, yt2, 'b', t, ye2, 'r--', t, y2, 'g--');
xlabel('Number of Samples');
ylabel('Output');
title('Kalman Filter Response - Output 2');
legend('True', 'Filtered', 'Measured');
