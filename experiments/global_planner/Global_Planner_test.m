
mTR = [0.1:0.1:1.0];
VD = [0.1:0.1:1.0];
mCD = [0.5:0.25:3];

mTR_i = cell(1, 10);
VD_i = cell(1, 10);
mCD_i = cell(1, 10);

for i = 1:10
    len = [];
    simtime = [];
    smooth = [];

    tic
    for j = 1:10
        [l, s, sm] = planificador_global(mTR(j), 0.5, 1.5);
        len(end + 1) = l;
        simtime(end + 1) = s;
        smooth(end + 1) = sm;
    end
    mTR_i{i} = [len; simtime; smooth];

    len = [];
    simtime = [];
    smooth = [];
    for j = 1:10
        [l, s, sm] = planificador_global(0.5, VD(j), 1.5);
        len(end + 1) = l;
        simtime(end + 1) = s;
        smooth(end + 1) = sm;
    end
    VD_i{i} = [len; simtime; smooth];

    len = [];
    simtime = [];
    smooth = [];
    for j = 1:11
        [l, s, sm] = planificador_global(0.5, 0.5, mCD(j));
        len(end + 1) = l;
        simtime(end + 1) = s;
        smooth(end + 1) = sm;
    end
    mCD_i{i} = [len; simtime; smooth];
    toc
end

