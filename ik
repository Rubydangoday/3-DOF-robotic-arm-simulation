function sols = ik_3dof(x, y, z)
%IK_3DOF  Dong hoc nghich cho robot 3-DOF RRR, tra ve TAT CA nghiem toan hoc
%(chua loc theo gioi han goc khop). Dung ham filter_joint_limits de loc.
%   sols = ik_3dof(x, y, z)
%
%   Input: toa do dich (x,y,z) tinh bang mm.
%          x, y: trong he goc O0 (khong doi).
%          z:    tinh TU MAT DAT THUC TE (dong bo voi fk_3dof.m), vd
%                z=291.5 mm ung voi tu the duoi thang hoan toan.
%
%   Output: sols la ma tran Nx3, moi hang [th1 th2 th3] (do), N <= 4
%           (2 cau hinh vai trai/phai x 2 cau hinh khuyu up/down)

    d1 = 144;
    d2 = 19.6;
    a2 = 190;
    a3 = 230;
    BASE_HEIGHT = 147.5;   % phai giong het gia tri trong fk_3dof.m

    z0 = z - BASE_HEIGHT;  % quy ve he goc O0 (goc DH) truoc khi giai

    sols = zeros(0,3);

    r2 = x^2 + y^2;
    if r2 < d2^2 - 1e-9
        % Diem qua gan truc J1, khong co nghiem (nam trong "vung chet" do offset vai)
        return;
    end

    % Xu ly sai so so hoc khi r2 hoi nho hon d2^2 do lam tron
    r2 = max(r2, d2^2);
    A_candidates = [ sqrt(r2 - d2^2), -sqrt(r2 - d2^2) ];   % vai phai / vai trai

    zp = z0 - d1;    % offset doc theo huong "roi xuong" cua khop 2-3

    for k = 1:2
        A = A_candidates(k);

        c3 = (A^2 + zp^2 - a2^2 - a3^2) / (2*a2*a3);
        if abs(c3) > 1 + 1e-6
            continue;   % ngoai tam voi, khong co nghiem cho nhanh nay
        end
        c3 = min(max(c3, -1), 1);   % kep so ve [-1,1] tranh loi lam tron

        s3_options = [ sqrt(1 - c3^2), -sqrt(1 - c3^2) ];  % khuyu down / up

        th1 = atan2d(y, x) - atan2d(d2, A);
        th1 = mod(th1 + 180, 360) - 180;   % chuan hoa ve khoang (-180, 180]

        for j = 1:2
            s3 = s3_options(j);
            th3 = atan2d(s3, c3);
            th2 = atan2d(zp, A) + atan2d(a3*s3, a2 + a3*c3);

            sols(end+1, :) = [th1, th2, th3]; %#ok<AGROW>
        end
    end
end
