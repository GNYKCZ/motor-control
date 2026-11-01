function dq=clarke_park(uv,theta)
ab=[uv(:,1),(uv(:,1)+2*uv(:,2))/sqrt(3)];
dq=[ab(:,1).*cos(theta)+ab(:,2).*sin(theta),-ab(:,1).*sin(theta)+ab(:,2).*cos(theta)];
end
