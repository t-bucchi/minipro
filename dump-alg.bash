#!/usr/bin/env bash

# This script is a part of the minipro project
# https://gitlab.com/DavidGriffith/minipro/

# This script downloads a .rar file containing the official Xgpro
# software.  This .rar file is then expanded and the algorithm files
# within are processed to create an algorithm.xml file for use with
# minipro.  This script is heavily based on and greatly expanded from a
# similar script by radiomanV <radioman_35@yahoo.com>.

# Why do this?  It's because for the T56 programmer, the bitstreams
# required by the programmer's FPGA are not contained in the programmer
# itself, but must be loaded from the controlling program.  The other
# Xgecu programmers keep their FPGA bitstreams in their own hardware.

# See
# https://hackaday.com/2021/01/29/why-blobs-are-important-and-why-you-should-care/
# for a discussion of this practice and why it's bad.

# Prerequisites:
#   bash, bsdtar, md5sum, wget or curl, and base64.

XGPRO_RAR=xgproV1267_setup.rar
XGPRO_RAR2=XgproV1267_Setup.exe
XGPRO_URL=https://github.com/Kreeblah/XGecu_Software/raw/refs/heads/master/Xgpro/12/
XGPRO_MD5=3f3411db1db471b337e90683995182b1

WORKDIR=`pwd`
XGPRO_ALG="algorithm"
ALG_FILENAME="algorithm.xml"

FIRMWARE_NAMES=("updateII.dat" "UpdateT48.dat" "updateT56.dat")
FIRMWARE_TYPE=("TL866II+" "T48" "T56")
FIRMWARE_VERSIONS=("04.2.105" "01.1.31" "01.1.72")
FIRMWARE_COUNT=3

#ALG_OFFSET=545		# 0x221
ALG_OFFSET=544		# 0x220

TAR="bsdtar"

usage() {
	echo "usage: $0 [ <path> ]" && grep " .)\ #" $0;
	echo
	echo "This script downloads Xgecu software RAR file to extract firmware and T56"
	echo "algorithms.  By default, this script will process the *.alg files in"
	echo "$XGPRO_RAR and put the resulting $ALG_FILENAME file in the current"
	echo "directory along with the latest firmware images for the TL866II+, T48, and"
	echo "T56 programmers."
	echo
	echo "If a path is supplied (for example: /usr/local/share/minipro/), this script"
	echo "will attempt to install $ALG_FILENAME there.  This is intended for use by the"
	echo "minipro Makefile and package management."
	echo
	echo "Prerequisite programs: bash, bsdtar, base64, and wget or curl.  If you don't"
	echo "see \"bsdtar\" as a separate package, try \"libarchive-tools\".  As the name"
	echo "suggests, on *BSD systems, your regular tar program is bsdtar."
	echo
	exit 0;
}

if [ $1 ] ; then
	if [ $1 = "help" ] || [ $1 = "-h" ]  ; then
		usage
		exit
	fi
fi

echo "Starting the Xgecu downloader."
echo "** Where to put the T56 algorithms ($ALG_FILENAME)?"

if [ $1 ]; then
	if ! [ -w $WORKDIR ]; then
		echo "** Error: Unable to write to $WORKDIR.  Fix that and try again."
		exit
	fi

	INSTALL_DIR=$1
	# Do we have write access now?
	# No?  Then see if we can do it through sudo.
	if ! [ -w $INSTALL_DIR ]; then
		if ! [ -x "$(command -v sudo)" ]; then
			echo "** Warning: sudo not found.  Install algorithms manually."
		elif [ !  $(command sudo test -w $INSTALL_DIR) ] ; then
			echo "** Error: Unable to write to $INSTALL_DIR.  Install minipro first."
			exit
		fi  # We can write with sudo
	fi  # We can write without needing root access
else
	INSTALL_DIR=$WORKDIR
fi
echo "    $INSTALL_DIR"


# Check for needed programs: bsdtar, md5sum, wget or curl, and base64.
#
if ! [ -x "$(command -v $TAR)" ]; then
        # Bsdtar not found.  Check if tar is really bsdtar
        TARVER="$(command -v "tar" --version | cut -d" " -f1 | head -n1)"
        if [ $TARVER == "tar" ]; then
                echo "** Good!  tar is really bsdtar."
		TAR="tar"
        else
                echo "** Error: bsdtar not installed." >&2
		echo "** Try libarchive-tools if you don't see a bsdtar package."
		ERROR=1
        fi
else
        echo "** Found bsdtar."
fi

if [ -x "$(command -v wget )" ]; then
	GETURL="wget -O"
	echo "** Found wget."
elif [ -x "$(command -v curl)" ]; then
	GETURL="curl -o"
	echo "** Found curl."
else
	echo "Error: Neither wget nor curl are installed." >&2
	ERROR=1
fi

if ! [ -x "$(command -v md5sum)" ]; then
	echo "Error: md5sum is not installed.  This should not happen." >&2
	ERROR=1
else
	echo "** Found md5sum."
fi

if ! [ -x "$(command -v base64)" ]; then
	echo "Error: base64 is not installed.  This should not happen." >&2
	ERROR=1
else
	echo "** Found base64."
fi

if [ ${ERROR} ] ; then
	echo "Install prerequsites and then try again." >&2
	exit 1
fi


# Download, check, then extract...
if [ ! -f "$WORKDIR/$XGPRO_RAR" ] ; then
	echo "** Downloading $XGPRO_RAR..."
	$GETURL $WORKDIR/$XGPRO_RAR $XGPRO_URL/$XGPRO_RAR
else
	echo "** $XGPRO_RAR already in $WORKDIR."
fi

echo "** Checking $XGPRO_RAR..."
MD5_CHECK=`md5sum $XGPRO_RAR | cut -d" " -f1`
if [ $XGPRO_MD5 != $MD5_CHECK ] ; then
	echo "** Error: MD5 checksum for $XGPRO_RAR is wrong."
	echo "** $XGPRO_MD5 != $MD5_CHECK"
	exit 2
fi
echo "** MD5 checksum is good."

echo "** Extracting..."
$TAR -x --to-stdout -f $WORKDIR/$XGPRO_RAR \
	| $TAR -x -f -  ${FIRMWARE_NAMES[*]} $XGPRO_ALG/*.alg

# Rename firmware files to reflect their versions.
#
NAME_INDEX=0
while [ $NAME_INDEX -lt $FIRMWARE_COUNT ] ; do
	NEWNAME="firmware-${FIRMWARE_TYPE[$NAME_INDEX]}-${FIRMWARE_VERSIONS[$NAME_INDEX]}.dat"
	echo "    ${FIRMWARE_NAMES[$NAME_INDEX]} --> $NEWNAME"
	mv ${FIRMWARE_NAMES[$NAME_INDEX]} $NEWNAME
	NAME_INDEX=$((NAME_INDEX+1))
done
echo "    $XGPRO_ALG/"


# Extracting algorithm data and building algorithm.xml.
#
echo "** Processing T56 algorithm files to create $ALG_FILENAME..."
echo '<root>' > $ALG_FILENAME
echo '<database type="ALGORITHMS">' >> $ALG_FILENAME
echo '<algorithms_T56>' >> $ALG_FILENAME
TICK_COUNT=0
TICK_MAX=75
for file in  $XGPRO_ALG/*.alg  ; do
	if [ $TICK_COUNT -le $TICK_MAX ] ; then
		TICK_COUNT=$((TICK_COUNT+1))
		echo -n "."
	else
		TICK_COUNT=0
		echo "."
	fi
	read -d "" desc < $file
	bitstream=$(tail -c +$ALG_OFFSET $file | gzip --best | base64 -w 0)
	file=$(basename -- "$file")
	echo "<algorithm name=\"${file%.*}\"" >> $ALG_FILENAME
	echo "description=\"${desc}\"" >> $ALG_FILENAME
	echo "bitstream=\"${bitstream}\"/>" >> $ALG_FILENAME
done
echo '</algorithms_T56>' >> $ALG_FILENAME
echo '</database>' >> $ALG_FILENAME
echo '</root>' >> $ALG_FILENAME

rm -rf $XGPRO_ALG/

if [ $INSTALL_DIR != $WORKDIR ] ; then
	echo "** Installing $ALG_FILENAME to $INSTALL_DIR..."
fi
echo
echo "** Done!"
