GITUSER=sestre-oratek
GITTOKEN=ec0d0bd632dc497bb0e59be48e660462a1d72b2b

COMMITUSERNAME="Automatic realeser"
COMMITUSEREMAIL="hello@oratek.com"


#if the version number is not in parameters
if [ $# -eq 0 ]
then
  echo "Please give me a version number"
  exit 1
fi

echo ""
echo "----- prepare ssh key -----"
cp -a ./sshKeys/. ~/.ssh/
ssh-keyscan -H github.com >> ~/.ssh/known_hosts

echo ""
echo "----- create temp folder -----"
mkdir 'temp'
cd temp

echo ""
echo "----- clone repo -----"
git clone git@github.com:Sestre-oratek/oratek-avr-boards.git
cd oratek-avr-boards
git checkout master

echo ""
echo "----- creat new branch -----"
if git rev-parse --quiet --verify $1 > /dev/null
then
  git checkout $1
  NEWBRANCH=false
else
  git checkout -b $1
  NEWBRANCH=true
fi

##echo ""
##echo "----- apply changes -----"
##cp -rf ../../../oratek-avr-boards ..
##touch added.txt

echo ""
echo "----- commit changes -----"
git add .
git config user.name $COMMITUSERNAME
git config user.email $COMMITUSEREMAIL
git commit -m "release V$1"

echo ""
echo "----- push changes -----"
git push --set-upstream origin $1
cd ..
sleep 1

echo ""
echo "----- dl repo archive -----"
curl "https://codeload.github.com/Sestre-oratek/oratek-avr-boards/zip/$1" >> oratek-avr-boards.zip

echo ""
echo "----- calculate size and checksum -----"
SIZE=$(stat -c%s oratek-avr-boards.zip)
SUM=$(sha256sum oratek-avr-boards.zip | cut -d " " -f 1)
echo "size = ${SIZE}"
echo "sum = ${SUM}"

echo ""
echo "----- clone json file repo -----"
git clone https://github.com/Sestre-oratek/jsonFiles.git
cd jsonFiles

echo ""
echo "----- Add new version to the json file -----"
../../jq-win64.exe .version="\"$1\"" avrTemplate.json | ../../jq-win64.exe .size="\"${SIZE}\"" | ../../jq-win64.exe .checksum="\"${SUM}\"" | ../../jq-win64.exe .archiveFileName="\"oratek-avr-$1.zip\"" | ../../jq-win64.exe .url="\"https://github.com/Sestre-oratek/oratek-avr-boards/archive/$1\"" >> temp.json
../../jq-win64.exe --argjson newVersion "$(<temp.json)" '.packages[0].platforms += [$newVersion]' package_oratek_index.json >> final.json
rm temp.json
rm package_oratek_index.json
mv final.json package_oratek_index.json

echo ""
echo "----- commit json -----"
git add .
git config user.name $COMMITUSERNAME
git config user.email $COMMITUSEREMAIL
git commit -m "Add AVR V$1"

echo ""
echo "----- push json -----"
git push
cd ..

echo ""
echo "----- delete temp folder -----"
cd ..
rm -rf temp

echo ""
echo "----- END -----"
