go build -o planner
mv planner executive
cd executive
make
chmod 777 shim.py executive controler
cd -