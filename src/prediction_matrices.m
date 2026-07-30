function [Psi,Gamma,Theta,U0,Tu] = prediction_matrices(A,B,Hp,Hu)
% Matris haye pishbinie halat va voroodi

nx=size(A,1); nu=size(B,2);
Psi=zeros(Hp*nx,nx);
Gamma=zeros(Hp*nx,nu);
Theta=zeros(Hp*nx,Hu*nu);
U0=kron(ones(Hp,1),eye(nu));
Tu=zeros(Hp*nu,Hu*nu);

for i=1:Hp
    rows=(i-1)*nx+(1:nx);
    Psi(rows,:)=A^i;
    for j=1:i
        Gamma(rows,:)=Gamma(rows,:)+A^(i-j)*B;
        for m=1:min(j,Hu)
            cols=(m-1)*nu+(1:nu);
            Theta(rows,cols)=Theta(rows,cols)+A^(i-j)*B;
        end
    end
    for m=1:min(i,Hu)
        Tu((i-1)*nu+(1:nu),(m-1)*nu+(1:nu))=eye(nu);
    end
end
end
