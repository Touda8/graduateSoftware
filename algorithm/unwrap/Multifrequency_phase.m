function pha12 = Multifrequency_phase(pha1, pha2)
mask = pha1 > pha2;
mask2=(1 - mask);
pha12 = (pha1 - pha2) .* mask + (2 * pi - (pha2 - pha1)) .* mask2;
end