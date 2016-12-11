fdata = load('./sf/outcmafit.dat')
iter = fdata(:,1);
bestever = fdata(:,6);

figure;
plot(iter, bestever)
ylabel('bestever')
xlabel('iteration')