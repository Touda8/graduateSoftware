function unwrappedPhase = UNwrapPhase(keyPhase, multiple, wrappedPhase)
    multiple_1 = round((multiple .* keyPhase - wrappedPhase)/(2*pi));
    unwrappedPhase = wrappedPhase + multiple_1.*(2*pi);
end