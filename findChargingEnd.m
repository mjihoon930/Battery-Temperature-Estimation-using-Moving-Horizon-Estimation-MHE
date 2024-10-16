
function index_end = findChargingEnd(Current_A, index_pos,Charge_Curr)
positive_indices = find(Current_A(index_pos) < -0.99*Charge_Curr);
if any(diff(positive_indices)>1)
    positive_indices=positive_indices(1:find(diff(positive_indices)>1));
end

if ~isempty(positive_indices)
    index_end = index_pos(1) + positive_indices(end) - 1;
else
    warning('No positive current found in this charge phase.');
    index_end = index_pos(end);
end
end