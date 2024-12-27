GENFLAGS = -std=c++20 -O3 -I./include
GENFILES = gen constraints path trajectory

.PHONY: gen

bin/:
	mkdir -p bin

$(patsubst %, bin/%.o, $(GENFILES)): bin/%.o: src/%.cpp | bin/
	g++ $(GENFLAGS) -c $< -o $@

bin/gen.exe: $(patsubst %, bin/%.o, $(GENFILES)) | bin/
	g++ $(GENFLAGS) -o $@ $^

gen: bin/gen.exe
	./bin/gen.exe > static/path.txt
