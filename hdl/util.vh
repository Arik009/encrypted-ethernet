function integer clog2(input integer size);
begin
	size = size - 1;
	for (clog2 = 1; size > 1; clog2 = clog2 + 1)
		size = size >> 1;
	end
endfunction
