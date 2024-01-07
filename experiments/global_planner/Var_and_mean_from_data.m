disp("NOW WITH mTR -----------------")

mytime = [];
len = [];
smoth = [];

for j=1:10
    for i=1:10
        mytime(end+1) = mTR_i{1,i}(1,j);
        len(end+1) = mTR_i{1,i}(2,j);
        smoth(end+1) = mTR_i{1,i}(3,j);
    end
    disp("iter = " + num2str(j));
    disp("mean time = " + num2str(mean(mytime)));
    disp("var time = " + num2str(var(mytime)));
    disp("mean len = " + num2str(mean(len)));
    disp("var len = " + num2str(var(len)));
    disp("mean smoth = " + num2str(mean(smoth)));
    disp("var smoth = " + num2str(var(smoth)));
end

disp("NOW WITH VD -----------------")

mytime = [];
len = [];
smoth = [];

for j=1:10
    for i=1:10
        mytime(end+1) = VD_i{1,i}(1,j);
        len(end+1) = VD_i{1,i}(2,j);
        smoth(end+1) = VD_i{1,i}(3,j);
    end
    disp("iter = " + num2str(j));
    disp("mean time = " + num2str(mean(mytime)));
    disp("var time = " + num2str(var(mytime)));
    disp("mean len = " + num2str(mean(len)));
    disp("var len = " + num2str(var(len)));
    disp("mean smoth = " + num2str(mean(smoth)));
    disp("var smoth = " + num2str(var(smoth)));
end

disp("NOW WITH mCD -----------------")

mytime = [];
len = [];
smoth = [];

for j=1:11
    for i=1:10
        mytime(end+1) = mCD_i{1,i}(1,j);
        len(end+1) = mCD_i{1,i}(2,j);
        smoth(end+1) = mCD_i{1,i}(3,j);
    end
    disp("iter = " + num2str(j));
    disp("mean time = " + num2str(mean(mytime)));
    disp("var time = " + num2str(var(mytime)));
    disp("mean len = " + num2str(mean(len)));
    disp("var len = " + num2str(var(len)));
    disp("mean smoth = " + num2str(mean(smoth)));
    disp("var smoth = " + num2str(var(smoth)));
end