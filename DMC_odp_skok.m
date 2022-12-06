close all;

load('odpowiedz.mat')

k1 = 489;
k2 = 700;

y = y(k1: k2);
u = u(k1:k2);

n= k2 - k1;

Upp=0;
Ypp=0;

u_step=1000;

for k=1:n
    s_step(k)=y(k)/u_step;
end

subplot(2, 1, 1)
stairs(y);
xlabel('$k$', 'Interpreter','latex');
ylabel('$y$', 'Interpreter','latex');
hold on
stairs(u)

subplot(2, 1, 2)
stairs(s_step);
xlabel('$k$', 'Interpreter','latex');
ylabel('$s$', 'Interpreter','latex');

yl = get(gca,'YTickLabel');
set(gca, 'YTickLabel', strrep(yl(:),'.',','))

set(groot,'defaultAxesTickLabelInterpreter','latex');  
set(gcf,'units','points','position',[100 100 450 300]);
% print('plots/zadanie_3/zad_3_odp_wej_wyj','-depsc','-r400');  % Zapisywanie wykresu
