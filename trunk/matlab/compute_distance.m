function total = compute_distance( M )
total = 0;
for k=1:size(M,1)-1
  total = total +  sqrt((M(k,1) - M(k+1,1))^2 + (M(k,2) -M(k+1,2))^2);
end
end

