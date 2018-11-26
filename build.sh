mkdir -p ~/go/src/github.com/afb2001/CCOM_planner/
cp -r parse util common dubins bitStar ~/go/src/github.com/afb2001/CCOM_planner/
go build -o planner
mv planner executive
cd executive
make
chmod 777 shim.py executive controler
cd -
