COMMITUSERNAME="OraBot"
COMMITUSEREMAIL="hello@oratek.com"
VERSIONNUMBERFILE=version


echo "----- Read version number file -----"
cd oratek-avr-boards
if [ -f "$VERSIONNUMBERFILE" ]
then
  VERSIONNUMBER=$(cat $VERSIONNUMBERFILE)
  echo "version number is $VERSIONNUMBER"
else 
  echo "$VERSIONNUMBERFILE does not exist."
  echo "Please give me a version number"
  exit 1
fi

echo ""
echo "----- Creat new branch -----"
EXISTS=$(git ls-remote --heads origin ${VERSIONNUMBER})
if [[ -z ${EXISTS} ]]
then
  git checkout -b $VERSIONNUMBER
else
  echo "Error: Version $VERSIONNUMBER already exists. Please change the version number in file \"$VERSIONNUMBERFILE\" or delete the branch $VERSIONNUMBER."
  exit 2
fi

echo ""
echo "----- Push changes -----"
git push --set-upstream origin $VERSIONNUMBER
cd ..
sleep 1

echo ""
echo "----- Download repo archive -----"
curl "https://codeload.github.com/oratek-ch/oratek-avr-boards/zip/$VERSIONNUMBER" >> oratek-avr-boards.zip

echo ""
echo "----- Calculate size and checksum -----"
SIZE=$(stat -c%s oratek-avr-boards.zip)
SUM=$(sha256sum oratek-avr-boards.zip | cut -d " " -f 1)
echo "size = ${SIZE}"
echo "sum = ${SUM}"

echo ""
echo "----- Add new version to the json file -----"
cd arduino-boards-manager
jq .version="\"$VERSIONNUMBER\"" avrTemplate.json | jq .size="\"${SIZE}\"" | jq .checksum="\"SHA-256:${SUM}\"" | jq .archiveFileName="\"oratek-avr-boards-$VERSIONNUMBER.zip\"" | jq .url="\"https://github.com/oratek-ch/oratek-avr-boards/archive/$VERSIONNUMBER.zip\"" >> temp.json
jq --argjson newVersion "$(<temp.json)" '.packages[0].platforms += [$newVersion]' package_oratek_index.json >> final.json
rm temp.json
rm package_oratek_index.json
mv final.json package_oratek_index.json
echo "done"

echo ""
echo "----- Commit json -----"
git add .
git config user.name $COMMITUSERNAME
git config user.email $COMMITUSEREMAIL
git commit -m "Add avr v$VERSIONNUMBER platform"

echo ""
echo "----- Push json -----"
git push

echo ""
echo "----- END -----"
