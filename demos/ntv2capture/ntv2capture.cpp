/* SPDX-License-Identifier: MIT */
/**
	@file		ntv2capture.cpp
	@brief		Implementation of NTV2Capture class.
	@copyright	(C) 2012-2022 AJA Video Systems, Inc.  All rights reserved.
**/

//#define AJA_RAW_AUDIO_RECORD	//	Uncomment to record captured raw audio into binary data file
//#define AJA_WAV_AUDIO_RECORD	//	Uncomment to record captured audio into WAV file
#include "ntv2capture.h"
#include "ancillarydata.h"
#include "ancillarylist.h"
#include "ntv2devicescanner.h"
#include "ntv2utils.h"
#include "ntv2devicefeatures.h"
#include "ajabase/system/process.h"
#include <iterator>	//	for inserter
#include <fstream>	//	for ofstream
#include <unistd.h>	//	for write
#include <stdint.h>	//	for uint32_t/uint64_t
#include <string.h>	//	for memcpy
#include <vector>
#include <stdlib.h>	//	for getenv
#include <algorithm>
#include <cctype>
#include <iomanip>
#include <map>
#include <cmath>
#include <set>
#include <sstream>
#include <fcntl.h>	//	for open
#include <sys/mman.h>
#include <sys/stat.h>

using namespace std;

#define NTV2_BUFFER_LOCK

static NTV2VideoFormat LevelBToLevelAFormat(const NTV2VideoFormat inFormat)
{
	switch (inFormat)
	{
		case NTV2_FORMAT_1080p_5000_B:	return NTV2_FORMAT_1080p_5000_A;
		case NTV2_FORMAT_1080p_5994_B:	return NTV2_FORMAT_1080p_5994_A;
		case NTV2_FORMAT_1080p_6000_B:	return NTV2_FORMAT_1080p_6000_A;
		case NTV2_FORMAT_1080p_2K_5000_B:	return NTV2_FORMAT_1080p_2K_5000_A;
		case NTV2_FORMAT_1080p_2K_5994_B:	return NTV2_FORMAT_1080p_2K_5994_A;
		case NTV2_FORMAT_1080p_2K_6000_B:	return NTV2_FORMAT_1080p_2K_6000_A;
		case NTV2_FORMAT_1080p_2K_4795_B:	return NTV2_FORMAT_1080p_2K_4795_A;
		case NTV2_FORMAT_1080p_2K_4800_B:	return NTV2_FORMAT_1080p_2K_4800_A;
		default:						return inFormat;
	}
}

namespace
{
	struct FrameTimecodeTelemetry
	{
		bool valid;
		int source;
		string timecode;
		string userBits;
		NTV2_RP188 raw;
	};

	struct AudioLTCProbeResult
	{
		string candidateChannels;
		string zeroCrossHz;
		string rmsDb;
		string confidence;
	};

	struct FrameCameraMetadataTelemetry
	{
		bool hasAnc;
		size_t ancBytes;
		uint32_t ancPackets;
		string cameraVendor;
		string cameraModel;
		string cameraID;
		string cameraSerial;
		string cameraFirmware;
		string lensData;
		string isoEI;
		string shutter;
		string whiteBalance;
		string tint;
		string nd;
		string aperture;
		string focusDistance;
		string cameraMediaFilename;
		string cameraClipName;
		string reelName;
		string codec;
		string recordingFormat;
		string sensorFPS;
		string projectFPS;
		string colorSpace;
		string gamma;
		string gamut;
		string lutName;
		string lookName;
		string anamorphicSqueeze;
		string recordFlag;
		string cameraTimecode;
		string cameraMetadataConfidence;
			string ancHash;
			string ancPacketIDs;
			string ancPayloadTypes;
			string ancTextPreview;
			string ancRawPrefixHex;
			string rdd18PayloadHex;
		string rdd18PacketIDs;
		string sdiKLVPacketIDs;
		string sdiKLVPayloadHex;
		string sdiKLVSummary;
		string arriMetadataPacketIDs;
		string arriMetadataPayloadHex;
		string arriMetadataSummary;
		string sonyMetadataPacketIDs;
		string sonyMetadataPayloadHex;
		string sonyMetadataSummary;
		string canonMetadataPacketIDs;
		string canonMetadataPayloadHex;
		string canonMetadataSummary;
		string redMetadataPacketIDs;
		string redMetadataPayloadHex;
		string redMetadataSummary;
		string blackmagicTallyPayloadHex;
		string blackmagicTallyPacketIDs;
		string blackmagicTallySummary;
		string blackmagicTallyMonitor;
		string blackmagicTallyVersion;
		string blackmagicTallySlaveStates;
		string blackmagicCCUPayloadHex;
		string blackmagicCCUPacketIDs;
		string blackmagicCCUSummary;
		string blackmagicCCUDecodedFields;
		string blackmagicVideoMode;
		string blackmagicRecordingFormat;
		string blackmagicCodec;
		string blackmagicTransportMode;
		string blackmagicDynamicRange;
		string blackmagicDisplayLUT;
		string blackmagicTimecodeSource;
		string blackmagicReference;
		string blackmagicAudio;
		string blackmagicOutput;
		string blackmagicDisplay;
		string blackmagicColorCorrection;
		string blackmagicConfiguration;
		string blackmagicPTZ;
	};

	struct KLVPacketFragment
	{
		uint8_t did;
		uint8_t sid;
		uint8_t messageID;
		uint16_t segmentIndex;
		vector<uint8_t> payload;
	};

	static string SanitizedMetadataValue (string value)
	{
		if (value.empty())
			return "-";
		for (size_t ndx(0); ndx < value.size(); ndx++)
		{
			unsigned char ch(static_cast<unsigned char>(value[ndx]));
			if (isspace(ch) || value[ndx] == '=' || value[ndx] == '"' || value[ndx] == '\'')
				value[ndx] = '_';
			else if (ch < 32)
				value[ndx] = '_';
		}
		if (value.size() > 180)
			value = value.substr(0, 180);
		return value;
	}

	static string LowercaseString (string value)
	{
		transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) { return static_cast<char>(tolower(ch)); });
		return value;
	}

	static bool ContainsCI (const string & haystack, const string & needle)
	{
		if (needle.empty())
			return false;
		return LowercaseString(haystack).find(LowercaseString(needle)) != string::npos;
	}

	static bool IsWordChar (const char ch)
	{
		return isalnum(static_cast<unsigned char>(ch)) || ch == '_';
	}

	static bool HasTokenBoundary (const string & value, const size_t pos, const size_t len)
	{
		if (pos > 0 && IsWordChar(value[pos - 1]))
			return false;
		const size_t after(pos + len);
		if (after < value.size() && IsWordChar(value[after]))
			return false;
		return true;
	}

	static string ExtractAfterLabel (const string & text, const vector<string> & labels, const size_t maxLen = 48)
	{
		const string lowerText(LowercaseString(text));
		for (vector<string>::const_iterator label(labels.begin()); label != labels.end(); ++label)
		{
			const string lowerLabel(LowercaseString(*label));
			size_t pos(lowerText.find(lowerLabel));
			while (pos != string::npos)
			{
				if (HasTokenBoundary(lowerText, pos, lowerLabel.size()))
				{
					size_t start(pos + lowerLabel.size());
					while (start < text.size())
					{
						const char ch(text[start]);
						if (ch == ':' || ch == '=' || ch == '-' || ch == '_' || ch == ' ' || ch == '\t')
							start++;
						else
							break;
					}
					string value;
					for (size_t ndx(start); ndx < text.size() && value.size() < maxLen; ndx++)
					{
						const char ch(text[ndx]);
						if (ch == '|' || ch == ';' || ch == '\n' || ch == '\r')
							break;
						if (isprint(static_cast<unsigned char>(ch)))
							value.push_back(ch);
					}
					while (!value.empty() && isspace(static_cast<unsigned char>(value.back())))
						value.erase(value.end() - 1);
					if (!value.empty())
						return SanitizedMetadataValue(value);
				}
				pos = lowerText.find(lowerLabel, pos + 1);
			}
		}
		return "-";
	}

	static string MatchFirstMarker (const string & text, const vector<string> & markers)
	{
		for (vector<string>::const_iterator it(markers.begin()); it != markers.end(); ++it)
			if (ContainsCI(text, *it))
				return SanitizedMetadataValue(*it);
		return "-";
	}

	static uint32_t UpdateFNV1a (uint32_t hash, const void * pData, const size_t byteCount)
	{
		const uint8_t * bytes(reinterpret_cast<const uint8_t *>(pData));
		for (size_t ndx(0); bytes && ndx < byteCount; ndx++)
		{
			hash ^= uint32_t(bytes[ndx]);
			hash *= 16777619u;
		}
		return hash;
	}

	static string HexValue (const uint32_t value, const unsigned width = 2)
	{
		ostringstream oss;
		oss << "0x" << hex << setw(width) << setfill('0') << value;
		return oss.str();
	}

	static string PayloadHex (const uint8_t * pData, const size_t byteCount, const size_t maxBytes = 64)
	{
		if (!pData || !byteCount)
			return "-";
		ostringstream oss;
		oss << "0x" << hex << setfill('0');
		const size_t count(min(byteCount, maxBytes));
		for (size_t ndx(0); ndx < count; ndx++)
			oss << setw(2) << unsigned(pData[ndx]);
		if (byteCount > maxBytes)
			oss << "...";
		return oss.str();
	}

	static void AppendPrintableRuns (const uint8_t * pData, const size_t byteCount, string & outText, const size_t maxChars = 4096)
	{
		string run;
		for (size_t ndx(0); pData && ndx < byteCount && outText.size() < maxChars; ndx++)
		{
			const unsigned char ch(pData[ndx]);
			if ((ch >= 32 && ch <= 126) || ch == '\t')
			{
				run.push_back(ch == '\t' ? ' ' : static_cast<char>(ch));
				continue;
			}
			if (run.size() >= 4)
			{
				if (!outText.empty())
					outText += " | ";
				outText += run;
			}
			run.clear();
		}
		if (run.size() >= 4 && outText.size() < maxChars)
		{
			if (!outText.empty())
				outText += " | ";
			outText += run;
		}
		if (outText.size() > maxChars)
			outText.resize(maxChars);
	}

	static string JoinSet (const set<string> & values, const size_t maxLen = 240)
	{
		string out;
		for (set<string>::const_iterator it(values.begin()); it != values.end(); ++it)
		{
			if (!out.empty())
				out += ",";
			if (out.size() + it->size() + 1 > maxLen)
			{
				out += "...";
				break;
			}
			out += *it;
		}
		return out.empty() ? "-" : SanitizedMetadataValue(out);
	}

	static string JoinVector (const vector<string> & values, const size_t maxLen = 240)
	{
		string out;
		for (size_t ndx(0); ndx < values.size(); ndx++)
		{
			if (values[ndx].empty() || values[ndx] == "-")
				continue;
			if (!out.empty())
				out += ",";
			if (out.size() + values[ndx].size() + 1 > maxLen)
			{
				out += "...";
				break;
			}
			out += values[ndx];
		}
		return out.empty() ? "-" : SanitizedMetadataValue(out);
	}

		static void AppendPacketID (string & value, const uint8_t did, const uint8_t sid)
		{
			if (value == "-")
				value.clear();
			if (!value.empty())
				value += ",";
			value += HexValue(did) + "/" + HexValue(sid);
		}

		static string DecimalValue (const double value, const int precision = 3);
		static string IntegerValue (const int64_t value);

			static bool MetadataValuePresent (const string & value)
			{
				return !value.empty() && value != "-";
			}

			static bool TextMetadataValuePresent (const string & value)
			{
				if (!MetadataValuePresent(value) || value.empty())
					return false;
				const unsigned char first(static_cast<unsigned char>(value.front()));
				if (!isalnum(first))
					return false;
				for (size_t ndx(0); ndx < value.size(); ndx++)
					if (isalnum(static_cast<unsigned char>(value[ndx])))
						return true;
				return false;
			}

		static void AssignIfPresent (string & target, const string & value)
		{
			if (MetadataValuePresent(value))
				target = value;
		}

		static string CleanMetadataString (string value)
		{
			while (!value.empty() && (value.front() == '\0' || isspace(static_cast<unsigned char>(value.front()))))
				value.erase(value.begin());
			while (!value.empty() && (value.back() == '\0' || isspace(static_cast<unsigned char>(value.back()))))
				value.erase(value.end() - 1);
			string out;
			for (size_t ndx(0); ndx < value.size(); ndx++)
			{
				const unsigned char ch(static_cast<unsigned char>(value[ndx]));
				if (!ch)
					break;
				if (isprint(ch))
					out.push_back(static_cast<char>(ch));
			}
			return out.empty() ? "-" : SanitizedMetadataValue(out);
		}

		static string MetadataStringAt (const vector<uint8_t> & data, const size_t offset, const size_t byteCount)
		{
			if (offset >= data.size())
				return "-";
			const size_t count(min(byteCount, data.size() - offset));
			return CleanMetadataString(string(reinterpret_cast<const char *>(&data[offset]), count));
		}

		static uint64_t ReadUnsignedBE (const vector<uint8_t> & data, const size_t offset, const size_t byteCount)
		{
			uint64_t value(0);
			for (size_t ndx(0); ndx < byteCount && offset + ndx < data.size(); ndx++)
				value = (value << 8) | uint64_t(data[offset + ndx]);
			return value;
		}

		static uint64_t ReadUnsignedLE (const vector<uint8_t> & data, const size_t offset, const size_t byteCount)
		{
			uint64_t value(0);
			for (size_t ndx(0); ndx < byteCount && offset + ndx < data.size(); ndx++)
				value |= uint64_t(data[offset + ndx]) << (8 * ndx);
			return value;
		}

		static int64_t SignExtend (uint64_t value, const size_t byteCount)
		{
			if (!byteCount || byteCount >= 8)
				return static_cast<int64_t>(value);
			const uint64_t signBit(uint64_t(1) << (byteCount * 8 - 1));
			if (value & signBit)
				value |= (~uint64_t(0)) << (byteCount * 8);
			return static_cast<int64_t>(value);
		}

		static int64_t ReadSignedAuto (const vector<uint8_t> & data, const size_t offset, const size_t byteCount, const int64_t plausibleMin, const int64_t plausibleMax)
		{
			const int64_t be(SignExtend(ReadUnsignedBE(data, offset, byteCount), byteCount));
			const int64_t le(SignExtend(ReadUnsignedLE(data, offset, byteCount), byteCount));
			if (be >= plausibleMin && be <= plausibleMax)
				return be;
			if (le >= plausibleMin && le <= plausibleMax)
				return le;
			return be;
		}

		static uint64_t ReadUnsignedAuto (const vector<uint8_t> & data, const size_t offset, const size_t byteCount, const uint64_t plausibleMin, const uint64_t plausibleMax)
		{
			const uint64_t be(ReadUnsignedBE(data, offset, byteCount));
			const uint64_t le(ReadUnsignedLE(data, offset, byteCount));
			if (be >= plausibleMin && be <= plausibleMax)
				return be;
			if (le >= plausibleMin && le <= plausibleMax)
				return le;
			return be;
		}

		static string ARRIProjectFPSName (const uint64_t value)
		{
			switch (value)
			{
				case 0: return "23.976";
				case 1: return "24.000";
				case 2: return "25.000";
				case 3: return "29.970";
				case 4: return "30.000";
				default: return "-";
			}
		}

		static string ARRINDName (const uint64_t filterType, const uint64_t density)
		{
			switch (filterType)
			{
				case 0: return "clear";
				case 1: return "ND_1.4";
				case 2: return "ND_0.6";
				case 3: return "ND_1.2";
				case 4: return "ND_2.1";
				case 5: return "ND_1.8";
				default:
					if (density > 0 && density < 10000)
						return "ND_" + DecimalValue(double(density) / 1000.0);
					return "-";
			}
		}

		static string ARRITimecodeAt (const vector<uint8_t> & data, const size_t offset)
		{
			if (offset + 4 > data.size())
				return "-";
			uint8_t bytes[4] = {data[offset], data[offset + 1], data[offset + 2], data[offset + 3]};
			auto decode = [](const uint8_t high, const uint8_t low) -> int {
				if (high > 9 || low > 9)
					return -1;
				return int(high) * 10 + int(low);
			};
			int hh(decode((bytes[0] >> 4) & 0x0f, bytes[0] & 0x0f));
			int mm(decode((bytes[1] >> 4) & 0x0f, bytes[1] & 0x0f));
			int ss(decode((bytes[2] >> 4) & 0x0f, bytes[2] & 0x0f));
			int ff(decode((bytes[3] >> 4) & 0x0f, bytes[3] & 0x0f));
			if (hh > 23 || mm > 59 || ss > 59 || ff > 99 || hh < 0 || mm < 0 || ss < 0 || ff < 0)
			{
				hh = decode((bytes[3] >> 4) & 0x0f, bytes[3] & 0x0f);
				mm = decode((bytes[2] >> 4) & 0x0f, bytes[2] & 0x0f);
				ss = decode((bytes[1] >> 4) & 0x0f, bytes[1] & 0x0f);
				ff = decode((bytes[0] >> 4) & 0x0f, bytes[0] & 0x0f);
			}
			if (hh > 23 || mm > 59 || ss > 59 || ff > 99 || hh < 0 || mm < 0 || ss < 0 || ff < 0)
				return "-";
			ostringstream oss;
			oss << setw(2) << setfill('0') << hh << ":"
				<< setw(2) << setfill('0') << mm << ":"
				<< setw(2) << setfill('0') << ss << ":"
				<< setw(2) << setfill('0') << ff;
			return oss.str();
		}

		static string ARRIPixelAspectAt (const vector<uint8_t> & data, const size_t offset)
		{
			if (offset + 4 > data.size())
				return "-";
			const uint32_t be(static_cast<uint32_t>(ReadUnsignedBE(data, offset, 4)));
			const uint32_t le(static_cast<uint32_t>(ReadUnsignedLE(data, offset, 4)));
			float beFloat(0), leFloat(0);
			memcpy(&beFloat, &be, sizeof(beFloat));
			memcpy(&leFloat, &le, sizeof(leFloat));
			if (isfinite(beFloat) && beFloat >= 0.5f && beFloat <= 3.0f)
				return DecimalValue(beFloat, 3) + "x";
			if (isfinite(leFloat) && leFloat >= 0.5f && leFloat <= 3.0f)
				return DecimalValue(leFloat, 3) + "x";
			const uint64_t scaled(ReadUnsignedAuto(data, offset, 4, 500, 3000));
			if (scaled >= 500 && scaled <= 3000)
				return DecimalValue(double(scaled) / 1000.0, 3) + "x";
			return "-";
		}

		static bool ApplyARRIMetadataBlock (const vector<uint8_t> & value, FrameCameraMetadataTelemetry & out)
		{
			if (value.size() < 0x06dd)
				return false;

			bool decoded(false);
			const string model(MetadataStringAt(value, 0x02f8, 20));
			const string firmware(MetadataStringAt(value, 0x02e0, 24));
			const string uuid(MetadataStringAt(value, 0x02d0, 16));
			const string reel(MetadataStringAt(value, 0x06b0, 11));
			const string clipName(MetadataStringAt(value, 0x06bb, 34));
			const string look(MetadataStringAt(value, 0x00cc, 32));
			const string lensModel(MetadataStringAt(value, 0x0398, 32));

			if (MetadataValuePresent(model) || MetadataValuePresent(reel) || MetadataValuePresent(clipName) || MetadataValuePresent(uuid))
			{
				out.cameraVendor = "ARRI";
				if (out.cameraMetadataConfidence == "-" || ContainsCI(out.cameraMetadataConfidence, "rp214"))
					out.cameraMetadataConfidence = "arri_rp214_klv_extended_metadata";
				decoded = true;
			}

			AssignIfPresent(out.cameraModel, model);
			AssignIfPresent(out.cameraFirmware, firmware);
			AssignIfPresent(out.cameraID, uuid);
			AssignIfPresent(out.reelName, reel);
			AssignIfPresent(out.cameraClipName, clipName);
			AssignIfPresent(out.cameraMediaFilename, clipName);
			AssignIfPresent(out.lookName, look);
			AssignIfPresent(out.lutName, look);
			AssignIfPresent(out.lensData, lensModel);

			const uint64_t cameraSerial(ReadUnsignedAuto(value, 0x0170, 4, 1, 999999999));
			if (cameraSerial > 0 && cameraSerial < 999999999)
				out.cameraSerial = IntegerValue(cameraSerial);

			const uint64_t whiteBalance(ReadUnsignedAuto(value, 0x005c, 4, 2000, 11000));
			if (whiteBalance >= 2000 && whiteBalance <= 11000)
				out.whiteBalance = IntegerValue(whiteBalance) + "K";

			const int64_t tint(ReadSignedAuto(value, 0x0060, 4, -1000, 1000));
			if (tint >= -1000 && tint <= 1000)
				out.tint = IntegerValue(tint);

			const uint64_t ei(ReadUnsignedAuto(value, 0x0074, 4, 1, 25600));
			if (ei > 0 && ei <= 25600)
				out.isoEI = IntegerValue(ei);

			const uint64_t shutterAngle(ReadUnsignedAuto(value, 0x0190, 4, 50, 3580));
			if (shutterAngle >= 50 && shutterAngle <= 3580)
				out.shutter = DecimalValue(double(shutterAngle) / 10.0, 1) + "deg";
			else
			{
				const uint64_t exposureUs(ReadUnsignedAuto(value, 0x018c, 4, 100, 100000));
				if (exposureUs >= 100 && exposureUs <= 100000)
					out.shutter = DecimalValue(double(exposureUs) / 1000.0, 3) + "ms";
			}

			const uint64_t sensorFPS(ReadUnsignedAuto(value, 0x01a0, 4, 750, 120000));
			if (sensorFPS >= 750 && sensorFPS <= 120000)
				out.sensorFPS = DecimalValue(double(sensorFPS) / 1000.0, 3);

			const uint64_t projectFPSIndex(ReadUnsignedAuto(value, 0x01a4, 4, 0, 4));
			AssignIfPresent(out.projectFPS, ARRIProjectFPSName(projectFPSIndex));
			AssignIfPresent(out.cameraTimecode, ARRITimecodeAt(value, 0x01a8));

			const uint64_t colorSpace(ReadUnsignedAuto(value, 0x00bc, 4, 0, 0xffffffffu));
			if (colorSpace == 0)
			{
				out.colorSpace = "Rec.709";
				out.gamma = "Rec.709";
				out.gamut = "Rec.709";
			}
			else if (colorSpace == 2)
			{
				out.colorSpace = "LogC/Wide_Gamut";
				out.gamma = "LogC";
				out.gamut = "ARRI_Wide_Gamut";
			}
			else if (colorSpace == 0x30323032u || colorSpace == 0x32303230u)
			{
				out.colorSpace = "Rec.2020";
				out.gamut = "Rec.2020";
			}
			AssignIfPresent(out.anamorphicSqueeze, ARRIPixelAspectAt(value, 0x00c4));

			const uint64_t focus(ReadUnsignedAuto(value, 0x0378, 4, 1, 100000000));
			const uint64_t focusUnit(ReadUnsignedAuto(value, 0x0374, 4, 0, 2));
			if (focus > 0 && focus < 100000000)
			{
				if (focusUnit == 0)
					out.focusDistance = DecimalValue(double(focus) / 1000.0, 3) + "in";
				else
					out.focusDistance = DecimalValue(double(focus) / 1000.0, 3) + "m";
			}

			const uint64_t focalLength(ReadUnsignedAuto(value, 0x037c, 4, 1, 500000));
			if (focalLength > 0 && focalLength <= 500000)
			{
				string focal = DecimalValue(double(focalLength) / 1000.0, 1) + "mm";
				if (MetadataValuePresent(out.lensData))
					out.lensData += "_" + focal;
				else
					out.lensData = focal;
			}

			const uint64_t irisRaw(ReadUnsignedAuto(value, 0x0384, 4, 1, 50000));
			if (irisRaw > 0 && irisRaw < 50000)
			{
				const double n((double(irisRaw) / 1000.0) - 1.0);
				const double tStop(pow(2.0, n / 2.0));
				if (isfinite(tStop) && tStop > 0.2 && tStop < 64.0)
					out.aperture = "T" + DecimalValue(tStop, 2);
			}

			const uint64_t ndType(ReadUnsignedAuto(value, 0x0388, 2, 0, 5));
			const uint64_t ndDensity(ReadUnsignedAuto(value, 0x038a, 2, 0, 10000));
			AssignIfPresent(out.nd, ARRINDName(ndType, ndDensity));

			return decoded;
		}

		static uint16_t ReadKLVSegmentIndex (const uint8_t * payload, const size_t payloadBytes)
		{
			if (!payload || payloadBytes < 3)
				return 0;
			const uint16_t be(uint16_t(payload[1]) << 8 | uint16_t(payload[2]));
			const uint16_t le(uint16_t(payload[2]) << 8 | uint16_t(payload[1]));
			if (be > 0 && be <= 128)
				return be;
			if (le > 0 && le <= 128)
				return le;
			if (payload[1] > 0 && payload[1] <= 128)
				return payload[1];
			if (payload[2] > 0 && payload[2] <= 128)
				return payload[2];
			return 0;
		}

		static void DecodeRP214KLVSummary (const vector<KLVPacketFragment> & fragments, FrameCameraMetadataTelemetry & out)
		{
			if (fragments.empty())
				return;

			map<unsigned, vector<KLVPacketFragment> > groups;
			for (size_t ndx(0); ndx < fragments.size(); ndx++)
			{
				groups[unsigned(fragments[ndx].messageID)].push_back(fragments[ndx]);
				AppendPacketID(out.sdiKLVPacketIDs, fragments[ndx].did, fragments[ndx].sid);
			}
			if (!fragments[0].payload.empty())
				out.sdiKLVPayloadHex = PayloadHex(&fragments[0].payload[0], fragments[0].payload.size());

			unsigned selectedMID(groups.begin()->first);
			size_t selectedCount(0);
			for (map<unsigned, vector<KLVPacketFragment> >::iterator it(groups.begin()); it != groups.end(); ++it)
				if (it->second.size() > selectedCount)
					{selectedMID = it->first; selectedCount = it->second.size();}

			vector<KLVPacketFragment> selected(groups[selectedMID]);
			sort(selected.begin(), selected.end(), [](const KLVPacketFragment & lhs, const KLVPacketFragment & rhs) {
				return lhs.segmentIndex < rhs.segmentIndex;
			});

			vector<uint8_t> value;
			uint64_t declaredLength(0);
			bool hasFirst(false);
			if (!selected.empty() && selected[0].segmentIndex == 1 && selected[0].payload.size() >= 23)
			{
				hasFirst = true;
				vector<uint8_t> first(selected[0].payload);
				declaredLength = ReadUnsignedBE(first, 19, 4);
				const uint64_t declaredLE(ReadUnsignedLE(first, 19, 4));
				if (declaredLength == 0 || declaredLength > 65536)
					declaredLength = declaredLE;
				value.insert(value.end(), first.begin() + 23, first.end());
				for (size_t ndx(1); ndx < selected.size(); ndx++)
				{
					if (selected[ndx].payload.size() <= 3)
						continue;
					value.insert(value.end(), selected[ndx].payload.begin() + 3, selected[ndx].payload.end());
				}
				if (declaredLength > 0 && declaredLength <= value.size())
					value.resize(static_cast<size_t>(declaredLength));
			}

			ostringstream summary;
			summary << "rp214_klv,mid:" << selectedMID
					<< ",packets:" << DEC(selected.size());
			if (hasFirst)
				summary << ",value_bytes:" << DEC(value.size())
						<< ",declared:" << DEC(declaredLength);

			if (out.cameraMetadataConfidence == "-")
				out.cameraMetadataConfidence = "rp214_klv_metadata";
			out.sdiKLVSummary = summary.str();

			if (value.size() >= 0x06dd && ApplyARRIMetadataBlock(value, out))
			{
				out.arriMetadataSummary = summary.str() + ",decoded:arri_extended_metadata";
				out.arriMetadataPayloadHex = PayloadHex(&value[0], value.size());
				for (size_t ndx(0); ndx < selected.size(); ndx++)
					AppendPacketID(out.arriMetadataPacketIDs, selected[ndx].did, selected[ndx].sid);
			}
		}

		static string KnownAncillaryTypeName (const uint8_t did, const uint8_t sid)
		{
			if (did == 0x51 && sid == 0x52)
				return "Blackmagic_Embedded_Tally";
			if (did == 0x51 && sid == 0x53)
				return "Blackmagic_SDI_Camera_Control";
			if (did == 0x52 && sid == 0x4d)
				return "Canon_Sony_Frame_Status";
			if (did == 0x52 && sid == 0x51)
				return "Canon_Frame_Status";
			if (did == 0x44 && (sid == 0x04 || sid == 0x14))
				return "RP214_KLV_Metadata";
			return SanitizedMetadataValue(AJAAncillaryData::DIDSIDToString(did, sid));
		}

	static string DecodeBlackmagicTallySummary (const uint8_t * payload, const size_t payloadBytes)
	{
		if (!payload || !payloadBytes)
			return "-";

		const uint8_t monitor(payload[0]);
		size_t slaveProgramCount(0);
		size_t slavePreviewCount(0);
		for (size_t ndx(1); ndx < payloadBytes; ndx++)
		{
			const uint8_t lower(payload[ndx] & 0x0f);
			const uint8_t upper((payload[ndx] >> 4) & 0x0f);
			if (lower & 0x01)
				slaveProgramCount++;
			if (lower & 0x02)
				slavePreviewCount++;
			if (upper & 0x01)
				slaveProgramCount++;
			if (upper & 0x02)
				slavePreviewCount++;
		}

		ostringstream oss;
		oss << "monitor_program:" << ((monitor & 0x01) ? "on" : "off")
			<< ",monitor_preview:" << ((monitor & 0x02) ? "on" : "off")
			<< ",version:" << unsigned((monitor >> 4) & 0x0f)
			<< ",slave_program:" << DEC(slaveProgramCount)
			<< ",slave_preview:" << DEC(slavePreviewCount)
			<< ",bytes:" << DEC(payloadBytes);
		return oss.str();
	}

	static string OnOff (const bool value)
	{
		return value ? "on" : "off";
	}

	static string BlackmagicTallyStateName (const bool program, const bool preview)
	{
		if (program && preview)
			return "program+preview";
		if (program)
			return "program";
		if (preview)
			return "preview";
		return "off";
	}

	static void ApplyBlackmagicTallyPayload (const uint8_t * payload, const size_t payloadBytes, FrameCameraMetadataTelemetry & out)
	{
		if (!payload || !payloadBytes)
			return;

		const uint8_t monitor(payload[0]);
		const bool monitorProgram((monitor & 0x01) != 0);
		const bool monitorPreview((monitor & 0x02) != 0);
		const uint8_t version((monitor >> 4) & 0x0f);
		size_t slaveProgramCount(0);
		size_t slavePreviewCount(0);
		vector<string> activeSlaveStates;
		for (size_t byteIndex(1); byteIndex < payloadBytes; byteIndex++)
		{
			const uint8_t pairs(payload[byteIndex]);
			for (size_t pair(0); pair < 2; pair++)
			{
				const uint8_t nibble((pairs >> (pair ? 4 : 0)) & 0x0f);
				const bool program((nibble & 0x01) != 0);
				const bool preview((nibble & 0x02) != 0);
				const size_t slaveID(((byteIndex - 1) * 2) + pair + 1);
				if (program)
					slaveProgramCount++;
				if (preview)
					slavePreviewCount++;
				if (program || preview)
					activeSlaveStates.push_back("slave" + IntegerValue(slaveID) + ":" + BlackmagicTallyStateName(program, preview));
			}
		}

		out.blackmagicTallyMonitor = BlackmagicTallyStateName(monitorProgram, monitorPreview);
		out.blackmagicTallyVersion = IntegerValue(version);
		out.blackmagicTallySlaveStates = activeSlaveStates.empty() ? "none" : JoinVector(activeSlaveStates);

		ostringstream oss;
		oss << "monitor:" << out.blackmagicTallyMonitor
			<< ",monitor_program:" << OnOff(monitorProgram)
			<< ",monitor_preview:" << OnOff(monitorPreview)
			<< ",version:" << unsigned(version)
			<< ",slave_program:" << DEC(slaveProgramCount)
			<< ",slave_preview:" << DEC(slavePreviewCount)
			<< ",slave_active:" << out.blackmagicTallySlaveStates
			<< ",slave_slots:" << DEC(payloadBytes > 1 ? (payloadBytes - 1) * 2 : 0)
			<< ",bytes:" << DEC(payloadBytes);
		out.blackmagicTallySummary = oss.str();
	}

	static int64_t ReadSignedLE (const uint8_t * data, const size_t byteCount)
	{
		if (!data || !byteCount)
			return 0;
		uint64_t raw(0);
		const size_t count(min<size_t>(byteCount, 8));
		for (size_t ndx(0); ndx < count; ndx++)
			raw |= uint64_t(data[ndx]) << (8 * ndx);
		if (count < 8 && (data[count - 1] & 0x80))
			raw |= (~uint64_t(0)) << (8 * count);
		return static_cast<int64_t>(raw);
	}

		static string DecimalValue (const double value, const int precision)
	{
		ostringstream oss;
		oss << fixed << setprecision(precision) << value;
		string out(oss.str());
		while (out.size() > 1 && out[out.size() - 1] == '0')
			out.erase(out.end() - 1);
		if (!out.empty() && out[out.size() - 1] == '.')
			out.erase(out.end() - 1);
		return out;
	}

	static string IntegerValue (const int64_t value)
	{
		ostringstream oss;
		oss << value;
		return oss.str();
	}

	static string BlackmagicGroupName (const uint8_t category)
	{
		switch (category)
		{
			case 0: return "lens";
			case 1: return "video";
			case 2: return "audio";
			case 3: return "output";
			case 4: return "display";
			case 5: return "tally";
			case 6: return "reference";
			case 7: return "configuration";
			case 8: return "color_correction";
			case 10: return "media";
			case 11: return "ptz";
			default: return "category_" + HexValue(category);
		}
	}

	static string BlackmagicParameterName (const uint8_t category, const uint8_t parameter)
	{
		if (category == 0)
		{
			switch (parameter)
			{
				case 0: return "focus";
				case 1: return "instant_autofocus";
				case 2: return "aperture_f_stop";
				case 3: return "aperture_normalized";
				case 4: return "aperture_ordinal";
				case 5: return "instant_auto_aperture";
				case 6: return "optical_image_stabilisation";
				case 7: return "zoom_mm";
				case 8: return "zoom_normalized";
				case 9: return "zoom_speed";
			}
		}
		else if (category == 1)
		{
			switch (parameter)
			{
				case 0: return "video_mode";
				case 1: return "gain_legacy";
				case 2: return "manual_white_balance";
				case 3: return "set_auto_white_balance";
				case 4: return "restore_auto_white_balance";
				case 5: return "exposure_us";
				case 6: return "exposure_ordinal";
				case 7: return "dynamic_range";
				case 8: return "video_sharpening";
				case 9: return "recording_format";
				case 10: return "auto_exposure_mode";
				case 11: return "shutter_angle";
				case 12: return "shutter_speed";
				case 13: return "gain_db";
				case 14: return "iso";
				case 15: return "display_lut";
				case 16: return "nd_filter_stop";
			}
		}
		else if (category == 5)
		{
			switch (parameter)
			{
				case 0: return "tally_brightness";
				case 1: return "front_tally_brightness";
				case 2: return "rear_tally_brightness";
			}
		}
		else if (category == 7)
		{
			switch (parameter)
			{
				case 0: return "real_time_clock";
				case 1: return "system_language";
				case 2: return "timezone";
				case 3: return "location";
			}
		}
		else if (category == 8)
		{
			switch (parameter)
			{
				case 0: return "lift";
				case 1: return "gamma";
				case 2: return "gain";
				case 3: return "offset";
				case 4: return "contrast";
				case 5: return "luma_mix";
				case 6: return "color";
				case 7: return "correction_reset_default";
			}
		}
		else if (category == 10)
		{
			switch (parameter)
			{
				case 0: return "codec";
				case 1: return "transport_mode";
				case 2: return "playback_control";
				case 5: return "stream";
				case 6: return "stream_information";
				case 7: return "stream_display_3d_lut";
			}
		}
		else if (category == 11)
		{
			switch (parameter)
			{
				case 0: return "pan_tilt_velocity";
				case 1: return "memory_preset";
			}
		}
		return BlackmagicGroupName(category) + "." + IntegerValue(parameter);
	}

	static size_t BlackmagicDataElementSize (const uint8_t dataType)
	{
		switch (dataType)
		{
			case 0: return 1;
			case 1: return 1;
			case 2: return 2;
			case 3: return 4;
			case 4: return 8;
			case 128: return 2;
			default: return 1;
		}
	}

	static string BlackmagicDecodedValue (const uint8_t dataType, const uint8_t * data, const size_t byteCount)
	{
		if (!data || !byteCount)
			return dataType == 0 ? "trigger" : "-";
		if (dataType == 0)
			return data[0] ? "true" : "false";
		if (dataType == 5)
			return SanitizedMetadataValue(string(reinterpret_cast<const char *>(data), byteCount));
		if (dataType == 128)
			return DecimalValue(double(ReadSignedLE(data, min<size_t>(byteCount, 2))) / 2048.0);
		const size_t elementSize(BlackmagicDataElementSize(dataType));
		vector<string> values;
		for (size_t offset(0); offset + elementSize <= byteCount && values.size() < 8; offset += elementSize)
			values.push_back(IntegerValue(ReadSignedLE(data + offset, elementSize)));
		if (values.empty())
			return "-";
		string out;
		for (size_t ndx(0); ndx < values.size(); ndx++)
		{
			if (ndx)
				out += "/";
			out += values[ndx];
		}
		if (byteCount / elementSize > values.size())
			out += "/...";
		return out;
	}

	static string BlackmagicFieldValue (const uint8_t dataType, const uint8_t * data, const size_t byteCount, const size_t elementIndex = 0)
	{
		if (!data || !byteCount)
			return "-";
		if (dataType == 5)
			return SanitizedMetadataValue(string(reinterpret_cast<const char *>(data), byteCount));
		const size_t elementSize(BlackmagicDataElementSize(dataType));
		const size_t offset(elementIndex * elementSize);
		if (offset + elementSize > byteCount)
			return "-";
		if (dataType == 0)
			return data[offset] ? "true" : "false";
		if (dataType == 128)
			return DecimalValue(double(ReadSignedLE(data + offset, elementSize)) / 2048.0);
		return IntegerValue(ReadSignedLE(data + offset, elementSize));
	}

	static bool BlackmagicFieldInt (const uint8_t dataType, const uint8_t * data, const size_t byteCount, const size_t elementIndex, int64_t & outValue)
	{
		if (!data || !byteCount || dataType == 5)
			return false;
		const size_t elementSize(BlackmagicDataElementSize(dataType));
		const size_t offset(elementIndex * elementSize);
		if (offset + elementSize > byteCount)
			return false;
		if (dataType == 0)
			outValue = data[offset] ? 1 : 0;
		else if (dataType == 128)
			outValue = ReadSignedLE(data + offset, elementSize);
		else
			outValue = ReadSignedLE(data + offset, elementSize);
		return true;
	}

	static void AppendBlackmagicState (string & target, const string & item)
	{
		if (!MetadataValuePresent(item))
			return;
		if (target == "-")
			target.clear();
		if (!target.empty())
			target += ";";
		target += item;
		if (target.size() > 180)
			target = target.substr(0, 176) + "...";
	}

	static string BlackmagicFrameRateName (const int64_t fps, const bool mRate)
	{
		if (fps <= 0)
			return "-";
		const double value(mRate ? (double(fps) * 1000.0 / 1001.0) : double(fps));
		return DecimalValue(value, mRate ? 3 : 2) + "fps";
	}

	static string BlackmagicDimensionsName (const int64_t dimensions)
	{
		switch (dimensions)
		{
			case 0: return "NTSC";
			case 1: return "PAL";
			case 2: return "720";
			case 3: return "1080";
			case 4: return "2K_DCI";
			case 5: return "2K_16x9";
			case 6: return "UHD";
			case 7: return "3K_Anamorphic";
			case 8: return "4K_DCI";
			case 9: return "4K_16x9";
			case 10: return "4.6K_2.4x1";
			case 11: return "4.6K";
			default: return "dimensions_" + IntegerValue(dimensions);
		}
	}

	static string BlackmagicDynamicRangeName (const int64_t mode)
	{
		switch (mode)
		{
			case 0: return "Film";
			case 1: return "Video";
			case 2: return "Extended_Video";
			default: return "dynamic_range_" + IntegerValue(mode);
		}
	}

	static string BlackmagicDisplayLUTName (const int64_t lut)
	{
		switch (lut)
		{
			case 0: return "None";
			case 1: return "Custom";
			case 2: return "Film_to_Video";
			case 3: return "Film_to_Extended_Video";
			default: return "lut_" + IntegerValue(lut);
		}
	}

	static string BlackmagicCodecName (const int64_t codec, const int64_t variant)
	{
		if (codec == 0)
		{
			switch (variant)
			{
				case 0: return "CinemaDNG_Uncompressed";
				case 1: return "CinemaDNG_3:1";
				case 2: return "CinemaDNG_4:1";
				default: return "CinemaDNG_variant_" + IntegerValue(variant);
			}
		}
		if (codec == 1)
			return variant >= 0 ? "DNxHD_variant_" + IntegerValue(variant) : "DNxHD";
		if (codec == 2)
		{
			switch (variant)
			{
				case 0: return "ProRes_HQ";
				case 1: return "ProRes_422";
				case 2: return "ProRes_LT";
				case 3: return "ProRes_Proxy";
				case 4: return "ProRes_444";
				case 5: return "ProRes_444XQ";
				default: return "ProRes_variant_" + IntegerValue(variant);
			}
		}
		if (codec == 3)
		{
			switch (variant)
			{
				case 0: return "Blackmagic_RAW_Q0";
				case 7: return "Blackmagic_RAW_Q1";
				case 8: return "Blackmagic_RAW_Q3";
				case 1: return "Blackmagic_RAW_Q5";
				case 2: return "Blackmagic_RAW_3:1";
				case 3: return "Blackmagic_RAW_5:1";
				case 4: return "Blackmagic_RAW_8:1";
				case 5: return "Blackmagic_RAW_12:1";
				default: return "Blackmagic_RAW_variant_" + IntegerValue(variant);
			}
		}
		return "codec_" + IntegerValue(codec) + "_variant_" + IntegerValue(variant);
	}

	static string BlackmagicTransportModeName (const int64_t mode)
	{
		switch (mode)
		{
			case 0: return "Preview";
			case 1: return "Play";
			case 2: return "Record";
			default: return "mode_" + IntegerValue(mode);
		}
	}

	static string BlackmagicStorageMediumName (const int64_t medium)
	{
		switch (medium)
		{
			case 0: return "CFast";
			case 1: return "SD";
			case 2: return "SSD_Recorder";
			default: return "medium_" + IntegerValue(medium);
		}
	}

	static string BlackmagicReferenceSourceName (const int64_t source)
	{
		switch (source)
		{
			case 0: return "Internal";
			case 1: return "Program";
			case 2: return "External";
			default: return "reference_" + IntegerValue(source);
		}
	}

	static string BlackmagicTimecodeSourceName (const int64_t source)
	{
		switch (source)
		{
			case 0: return "Clip";
			case 1: return "Timecode";
			default: return "source_" + IntegerValue(source);
		}
	}

	static string BlackmagicInputTypeName (const int64_t input)
	{
		switch (input)
		{
			case 0: return "Internal_Mic";
			case 1: return "Line_Level";
			case 2: return "Low_Mic_Level";
			case 3: return "High_Mic_Level";
			default: return "input_" + IntegerValue(input);
		}
	}

	static string BlackmagicAutoExposureModeName (const int64_t mode)
	{
		switch (mode)
		{
			case 0: return "Manual_Trigger";
			case 1: return "Iris";
			case 2: return "Shutter";
			case 3: return "Iris+Shutter";
			case 4: return "Shutter+Iris";
			default: return "auto_exposure_" + IntegerValue(mode);
		}
	}

	static string BlackmagicSharpeningName (const int64_t level)
	{
		switch (level)
		{
			case 0: return "Off";
			case 1: return "Low";
			case 2: return "Medium";
			case 3: return "High";
			default: return "sharpening_" + IntegerValue(level);
		}
	}

	static string BlackmagicNDDisplayModeName (const int64_t mode)
	{
		switch (mode)
		{
			case 0: return "Stop";
			case 1: return "Density";
			case 2: return "Transmittance";
			default: return "nd_display_" + IntegerValue(mode);
		}
	}

	static string BlackmagicTargetDisplayFlagsName (const int64_t flags)
	{
		vector<string> parts;
		if (flags & (1 << 0)) parts.push_back("LCD");
		if (flags & (1 << 1)) parts.push_back("HDMI");
		if (flags & (1 << 2)) parts.push_back("EVF");
		if (flags & (1 << 3)) parts.push_back("Main_SDI");
		if (flags & (1 << 4)) parts.push_back("Front_SDI");
		return parts.empty() ? "none" : JoinVector(parts);
	}

	static string BlackmagicOverlayFlagsName (const int64_t flags)
	{
		vector<string> parts;
		if (flags & (1 << 0)) parts.push_back("status");
		if (flags & (1 << 1)) parts.push_back("frame_guides");
		if (flags & (1 << 2)) parts.push_back("clean_feed");
		return parts.empty() ? "none" : JoinVector(parts);
	}

	static string BlackmagicExposureToolFlagsName (const int64_t flags)
	{
		vector<string> parts;
		if (flags & (1 << 0)) parts.push_back("zebra");
		if (flags & (1 << 1)) parts.push_back("focus_assist");
		if (flags & (1 << 2)) parts.push_back("false_color");
		return parts.empty() ? "none" : JoinVector(parts);
	}

	static string BlackmagicGridFlagsName (const int64_t flags)
	{
		vector<string> parts;
		if (flags & (1 << 0)) parts.push_back("thirds");
		if (flags & (1 << 1)) parts.push_back("cross_hairs");
		if (flags & (1 << 2)) parts.push_back("center_dot");
		if (flags & (1 << 3)) parts.push_back("horizon");
		return parts.empty() ? "none" : JoinVector(parts);
	}

	static string BlackmagicFrameGuideStyleName (const int64_t style)
	{
		switch (style)
		{
			case 0: return "Off";
			case 1: return "2.4:1";
			case 2: return "2.39:1";
			case 3: return "2.35:1";
			case 4: return "1.85:1";
			case 5: return "16:9";
			case 6: return "14:9";
			case 7: return "4:3";
			case 8: return "2:1";
			case 9: return "4:5";
			case 10: return "1:1";
			default: return "guide_" + IntegerValue(style);
		}
	}

	static string BlackmagicLegacyFrameGuideStyleName (const int64_t style)
	{
		switch (style)
		{
			case 0: return "HDTV";
			case 1: return "4:3";
			case 2: return "2.4:1";
			case 3: return "2.39:1";
			case 4: return "2.35:1";
			case 5: return "1.85:1";
			case 6: return "thirds";
			default: return "guide_" + IntegerValue(style);
		}
	}

	static string BlackmagicFocusAssistMethodName (const int64_t method)
	{
		switch (method)
		{
			case 0: return "Peak";
			case 1: return "Colored_Lines";
			default: return "method_" + IntegerValue(method);
		}
	}

	static string BlackmagicFocusAssistColorName (const int64_t color)
	{
		switch (color)
		{
			case 0: return "Red";
			case 1: return "Green";
			case 2: return "Blue";
			case 3: return "White";
			case 4: return "Black";
			default: return "color_" + IntegerValue(color);
		}
	}

	static string BlackmagicTransportFlagsName (const int64_t flags)
	{
		vector<string> parts;
		if (flags & (1 << 0))
			parts.push_back("loop");
		if (flags & (1 << 1))
			parts.push_back("play_all");
		if (flags & (1 << 5))
			parts.push_back("disk1_active");
		if (flags & (1 << 6))
			parts.push_back("disk2_active");
		if (flags & (1 << 7))
			parts.push_back("time_lapse");
		if (parts.empty())
			return "none";
		string out;
		for (size_t ndx(0); ndx < parts.size(); ndx++)
		{
			if (ndx)
				out += "+";
			out += parts[ndx];
		}
		return out;
	}

	static void ApplyBlackmagicCameraMetadataField (FrameCameraMetadataTelemetry & out,
												   const uint8_t category,
												   const uint8_t parameter,
												   const uint8_t dataType,
												   const uint8_t * data,
												   const size_t byteCount)
	{
		if (!data || (!byteCount && dataType != 0))
			return;
		const string parameterName(BlackmagicParameterName(category, parameter));
		const string decodedValue(BlackmagicDecodedValue(dataType, data, byteCount));
		const string stateItem(parameterName + ":" + decodedValue);
		if (category == 0)
		{
			if (parameter == 0)
				out.focusDistance = BlackmagicFieldValue(dataType, data, byteCount) + "_near_far";
			else if (parameter == 2)
			{
				int64_t rawAV(0);
				if (BlackmagicFieldInt(dataType, data, byteCount, 0, rawAV))
				{
					const double avValue(double(rawAV) / 2048.0);
					const double fNumber(sqrt(pow(2.0, avValue)));
					out.aperture = "f/" + DecimalValue(fNumber, 2) + " (AV " + DecimalValue(avValue, 2) + ")";
				}
				else
					out.aperture = BlackmagicFieldValue(dataType, data, byteCount);
			}
			else if (parameter == 3 || parameter == 4)
				out.aperture = BlackmagicFieldValue(dataType, data, byteCount);
			else if (parameter >= 6 && parameter <= 9)
				out.lensData = BlackmagicParameterName(category, parameter) + ":" + BlackmagicDecodedValue(dataType, data, byteCount);
		}
		else if (category == 1)
		{
			if (parameter == 0)
			{
				int64_t fps(0), mRate(0), dimensions(0), interlaced(0), colorSpace(0);
				const bool hasFPS(BlackmagicFieldInt(dataType, data, byteCount, 0, fps));
				BlackmagicFieldInt(dataType, data, byteCount, 1, mRate);
				BlackmagicFieldInt(dataType, data, byteCount, 2, dimensions);
				BlackmagicFieldInt(dataType, data, byteCount, 3, interlaced);
				BlackmagicFieldInt(dataType, data, byteCount, 4, colorSpace);
				const string fpsName(hasFPS ? BlackmagicFrameRateName(fps, mRate != 0) : "-");
				ostringstream oss;
				oss << "format:" << BlackmagicDimensionsName(dimensions)
					<< ",fps:" << fpsName
					<< ",scan:" << (interlaced ? "interlaced" : "progressive")
					<< ",color:" << (colorSpace == 0 ? "YUV" : ("color_" + IntegerValue(colorSpace)));
				out.blackmagicVideoMode = oss.str();
				if (fpsName != "-")
					out.projectFPS = fpsName;
				if (colorSpace == 0)
					out.colorSpace = "YUV";
			}
			else if (parameter == 2)
			{
				const string wb(BlackmagicFieldValue(dataType, data, byteCount, 0));
				const string tint(BlackmagicFieldValue(dataType, data, byteCount, 1));
				if (wb != "-")
					out.whiteBalance = wb + "K";
				if (tint != "-")
					out.tint = tint;
			}
			else if (parameter == 5)
				out.shutter = BlackmagicFieldValue(dataType, data, byteCount) + "us";
			else if (parameter == 7)
			{
				int64_t mode(0);
				if (BlackmagicFieldInt(dataType, data, byteCount, 0, mode))
				{
					out.blackmagicDynamicRange = BlackmagicDynamicRangeName(mode);
					if (mode == 0)
						out.gamma = "Blackmagic_Film";
					else
						out.gamma = out.blackmagicDynamicRange;
				}
			}
			else if (parameter == 8)
			{
				int64_t level(0);
				if (BlackmagicFieldInt(dataType, data, byteCount, 0, level))
					AppendBlackmagicState(out.blackmagicDisplay, "video_sharpening:" + BlackmagicSharpeningName(level));
			}
			else if (parameter == 9)
			{
				int64_t fileFPS(0), sensorFPS(0), width(0), height(0), flags(0);
				BlackmagicFieldInt(dataType, data, byteCount, 0, fileFPS);
				BlackmagicFieldInt(dataType, data, byteCount, 1, sensorFPS);
				BlackmagicFieldInt(dataType, data, byteCount, 2, width);
				BlackmagicFieldInt(dataType, data, byteCount, 3, height);
				BlackmagicFieldInt(dataType, data, byteCount, 4, flags);
				const string projectFPS(BlackmagicFrameRateName(fileFPS, (flags & (1 << 0)) != 0));
				const string sensorFPSName(BlackmagicFrameRateName(sensorFPS, (flags & (1 << 1)) != 0));
				vector<string> formatFlags;
				if (flags & (1 << 2))
					formatFlags.push_back("off_speed");
				if (flags & (1 << 3))
					formatFlags.push_back("interlaced");
				if (flags & (1 << 4))
					formatFlags.push_back("windowed");
				ostringstream oss;
				if (width > 0 && height > 0)
					oss << width << "x" << height;
				else
					oss << "format_unknown";
				if (projectFPS != "-")
					oss << "," << projectFPS;
				if (!formatFlags.empty())
				{
					oss << ",";
					for (size_t ndx(0); ndx < formatFlags.size(); ndx++)
					{
						if (ndx)
							oss << "+";
						oss << formatFlags[ndx];
					}
				}
				out.blackmagicRecordingFormat = oss.str();
				out.recordingFormat = out.blackmagicRecordingFormat;
				if (projectFPS != "-")
					out.projectFPS = projectFPS;
				if (sensorFPSName != "-")
					out.sensorFPS = sensorFPSName;
			}
			else if (parameter == 10)
			{
				int64_t mode(0);
				if (BlackmagicFieldInt(dataType, data, byteCount, 0, mode))
					AppendBlackmagicState(out.blackmagicConfiguration, "auto_exposure:" + BlackmagicAutoExposureModeName(mode));
			}
			else if (parameter == 11)
				out.shutter = DecimalValue(double(ReadSignedLE(data, min<size_t>(byteCount, 4))) / 100.0) + "deg";
			else if (parameter == 12)
				out.shutter = "1/" + BlackmagicFieldValue(dataType, data, byteCount);
			else if (parameter == 13)
				out.isoEI = BlackmagicFieldValue(dataType, data, byteCount) + "dB";
			else if (parameter == 14)
				out.isoEI = BlackmagicFieldValue(dataType, data, byteCount);
			else if (parameter == 15)
			{
				int64_t lut(0), enabled(0);
				BlackmagicFieldInt(dataType, data, byteCount, 0, lut);
				BlackmagicFieldInt(dataType, data, byteCount, 1, enabled);
				out.blackmagicDisplayLUT = BlackmagicDisplayLUTName(lut) + ",enabled:" + string(enabled ? "true" : "false");
				out.lutName = BlackmagicDisplayLUTName(lut);
			}
			else if (parameter == 16)
			{
				out.nd = BlackmagicFieldValue(dataType, data, byteCount, 0) + "_stops";
				int64_t displayMode(0);
				if (BlackmagicFieldInt(dataType, data, byteCount, 1, displayMode))
					AppendBlackmagicState(out.blackmagicDisplay, "nd_display:" + BlackmagicNDDisplayModeName(displayMode));
			}
		}
		else if (category == 2)
		{
			if (parameter == 4)
			{
				int64_t inputType(0);
				if (BlackmagicFieldInt(dataType, data, byteCount, 0, inputType))
					AppendBlackmagicState(out.blackmagicAudio, parameterName + ":" + BlackmagicInputTypeName(inputType));
			}
			else
				AppendBlackmagicState(out.blackmagicAudio, stateItem);
		}
		else if (category == 3)
		{
			if (parameter == 0)
			{
				int64_t flags(0), targets(0);
				BlackmagicFieldInt(dataType, data, byteCount, 0, flags);
				BlackmagicFieldInt(dataType, data, byteCount, 1, targets);
				AppendBlackmagicState(out.blackmagicOutput, "overlay:" + BlackmagicOverlayFlagsName(flags) + ",targets:" + BlackmagicTargetDisplayFlagsName(targets));
			}
			else if (parameter == 1)
			{
				int64_t style(0);
				if (BlackmagicFieldInt(dataType, data, byteCount, 0, style))
					AppendBlackmagicState(out.blackmagicOutput, parameterName + ":" + BlackmagicLegacyFrameGuideStyleName(style));
			}
			else if (parameter == 2)
				AppendBlackmagicState(out.blackmagicOutput, "overlay_opacity:" + BlackmagicFieldValue(dataType, data, byteCount));
			else if (parameter == 3)
			{
				int64_t style(0), opacity(0), safeArea(0), gridFlags(0);
				BlackmagicFieldInt(dataType, data, byteCount, 0, style);
				BlackmagicFieldInt(dataType, data, byteCount, 1, opacity);
				BlackmagicFieldInt(dataType, data, byteCount, 2, safeArea);
				BlackmagicFieldInt(dataType, data, byteCount, 3, gridFlags);
				AppendBlackmagicState(out.blackmagicOutput,
									  string("frame_guides:") + BlackmagicFrameGuideStyleName(style)
									  + ",opacity:" + IntegerValue(opacity)
									  + ",safe:" + IntegerValue(safeArea)
									  + ",grid:" + BlackmagicGridFlagsName(gridFlags));
			}
			else
				AppendBlackmagicState(out.blackmagicOutput, stateItem);
		}
		else if (category == 4)
		{
			if (parameter == 1)
			{
				int64_t flags(0), targets(0);
				BlackmagicFieldInt(dataType, data, byteCount, 0, flags);
				BlackmagicFieldInt(dataType, data, byteCount, 1, targets);
				AppendBlackmagicState(out.blackmagicDisplay, "tools:" + BlackmagicExposureToolFlagsName(flags) + ",targets:" + BlackmagicTargetDisplayFlagsName(targets));
			}
			else if (parameter == 2)
				AppendBlackmagicState(out.blackmagicDisplay, "zebra:" + BlackmagicFieldValue(dataType, data, byteCount) + "%");
			else if (parameter == 3)
				AppendBlackmagicState(out.blackmagicDisplay, "peaking_level:" + BlackmagicFieldValue(dataType, data, byteCount));
			else if (parameter == 4)
				AppendBlackmagicState(out.blackmagicDisplay, "color_bars_seconds:" + BlackmagicFieldValue(dataType, data, byteCount));
			else if (parameter == 5)
			{
				int64_t method(0), color(0);
				BlackmagicFieldInt(dataType, data, byteCount, 0, method);
				BlackmagicFieldInt(dataType, data, byteCount, 1, color);
				AppendBlackmagicState(out.blackmagicDisplay, "focus_assist:" + BlackmagicFocusAssistMethodName(method) + ",color:" + BlackmagicFocusAssistColorName(color));
			}
			else if (parameter == 6)
				AppendBlackmagicState(out.blackmagicDisplay, "program_return_seconds:" + BlackmagicFieldValue(dataType, data, byteCount));
			else if (parameter == 7)
			{
				int64_t source(0);
				if (BlackmagicFieldInt(dataType, data, byteCount, 0, source))
				{
					out.blackmagicTimecodeSource = BlackmagicTimecodeSourceName(source);
					AppendBlackmagicState(out.blackmagicDisplay, parameterName + ":" + out.blackmagicTimecodeSource);
				}
			}
			else
				AppendBlackmagicState(out.blackmagicDisplay, stateItem);
		}
		else if (category == 6)
		{
			if (parameter == 0)
			{
				int64_t source(0);
				if (BlackmagicFieldInt(dataType, data, byteCount, 0, source))
				{
					out.blackmagicReference = "source:" + BlackmagicReferenceSourceName(source);
				}
			}
			else if (parameter == 1)
				AppendBlackmagicState(out.blackmagicReference, "offset_pixels:" + BlackmagicFieldValue(dataType, data, byteCount));
		}
		else if (category == 7)
		{
			if (parameter == 0 && byteCount >= 8)
				AppendBlackmagicState(out.blackmagicConfiguration, string("rtc_time_bcd:") + PayloadHex(data, 4, 4) + ",date_bcd:" + PayloadHex(data + 4, 4, 4));
			else if (parameter == 1)
				AppendBlackmagicState(out.blackmagicConfiguration, "language:" + BlackmagicDecodedValue(dataType, data, byteCount));
			else if (parameter == 2)
				AppendBlackmagicState(out.blackmagicConfiguration, "timezone_minutes:" + BlackmagicFieldValue(dataType, data, byteCount));
			else if (parameter == 3)
				AppendBlackmagicState(out.blackmagicConfiguration, "location_bcd:" + PayloadHex(data, byteCount, 8));
			else
				AppendBlackmagicState(out.blackmagicConfiguration, stateItem);
		}
		else if (category == 8)
		{
			if (parameter == 0)
			{
				vector<string> values;
				values.push_back("lift_r:" + BlackmagicFieldValue(dataType, data, byteCount, 0));
				values.push_back("lift_g:" + BlackmagicFieldValue(dataType, data, byteCount, 1));
				values.push_back("lift_b:" + BlackmagicFieldValue(dataType, data, byteCount, 2));
				values.push_back("lift_luma:" + BlackmagicFieldValue(dataType, data, byteCount, 3));
				AppendBlackmagicState(out.blackmagicColorCorrection, JoinVector(values));
			}
			else if (parameter == 1)
			{
				vector<string> values;
				values.push_back("gamma_r:" + BlackmagicFieldValue(dataType, data, byteCount, 0));
				values.push_back("gamma_g:" + BlackmagicFieldValue(dataType, data, byteCount, 1));
				values.push_back("gamma_b:" + BlackmagicFieldValue(dataType, data, byteCount, 2));
				values.push_back("gamma_luma:" + BlackmagicFieldValue(dataType, data, byteCount, 3));
				AppendBlackmagicState(out.blackmagicColorCorrection, JoinVector(values));
			}
			else if (parameter == 2)
			{
				vector<string> values;
				values.push_back("gain_r:" + BlackmagicFieldValue(dataType, data, byteCount, 0));
				values.push_back("gain_g:" + BlackmagicFieldValue(dataType, data, byteCount, 1));
				values.push_back("gain_b:" + BlackmagicFieldValue(dataType, data, byteCount, 2));
				values.push_back("gain_luma:" + BlackmagicFieldValue(dataType, data, byteCount, 3));
				AppendBlackmagicState(out.blackmagicColorCorrection, JoinVector(values));
			}
			else if (parameter == 3)
			{
				vector<string> values;
				values.push_back("offset_r:" + BlackmagicFieldValue(dataType, data, byteCount, 0));
				values.push_back("offset_g:" + BlackmagicFieldValue(dataType, data, byteCount, 1));
				values.push_back("offset_b:" + BlackmagicFieldValue(dataType, data, byteCount, 2));
				values.push_back("offset_luma:" + BlackmagicFieldValue(dataType, data, byteCount, 3));
				AppendBlackmagicState(out.blackmagicColorCorrection, JoinVector(values));
			}
			else if (parameter == 4)
				AppendBlackmagicState(out.blackmagicColorCorrection, "contrast:" + BlackmagicFieldValue(dataType, data, byteCount, 0) + ",pivot:" + BlackmagicFieldValue(dataType, data, byteCount, 1));
			else if (parameter == 6)
				AppendBlackmagicState(out.blackmagicColorCorrection, "hue:" + BlackmagicFieldValue(dataType, data, byteCount, 0) + ",saturation:" + BlackmagicFieldValue(dataType, data, byteCount, 1));
			else
				AppendBlackmagicState(out.blackmagicColorCorrection, stateItem);
		}
		else if (category == 10)
		{
			if (parameter == 0)
			{
				int64_t codec(0), variant(0);
				if (BlackmagicFieldInt(dataType, data, byteCount, 0, codec))
				{
					BlackmagicFieldInt(dataType, data, byteCount, 1, variant);
					out.blackmagicCodec = BlackmagicCodecName(codec, variant);
					out.codec = out.blackmagicCodec;
				}
			}
			else if (parameter == 1)
			{
				int64_t mode(0), speed(0), flags(0), slot1(0), slot2(0);
				BlackmagicFieldInt(dataType, data, byteCount, 0, mode);
				BlackmagicFieldInt(dataType, data, byteCount, 1, speed);
				BlackmagicFieldInt(dataType, data, byteCount, 2, flags);
				BlackmagicFieldInt(dataType, data, byteCount, 3, slot1);
				BlackmagicFieldInt(dataType, data, byteCount, 4, slot2);
				ostringstream oss;
				oss << "mode:" << BlackmagicTransportModeName(mode)
					<< ",speed:" << IntegerValue(speed)
					<< ",flags:" << BlackmagicTransportFlagsName(flags)
					<< ",slot1:" << BlackmagicStorageMediumName(slot1)
					<< ",slot2:" << BlackmagicStorageMediumName(slot2);
				out.blackmagicTransportMode = oss.str();
				if (mode == 2)
					out.recordFlag = "true";
				else if (mode == 0 || mode == 1)
					out.recordFlag = "false";
			}
			else
				AppendBlackmagicState(out.blackmagicTransportMode, stateItem);
		}
		else if (category == 11)
			AppendBlackmagicState(out.blackmagicPTZ, stateItem);
	}

	static string DecodeBlackmagicCCUSummary (const uint8_t * payload, const size_t payloadBytes, FrameCameraMetadataTelemetry * outMetadata)
	{
		if (!payload || !payloadBytes)
			return "-";

		size_t offset(0);
		size_t messages(0);
		bool truncated(false);
		ostringstream samples;
		vector<string> decoded;
		while (offset + 4 <= payloadBytes)
		{
			const uint8_t destination(payload[offset]);
			const uint8_t commandLength(payload[offset + 1]);
			const uint8_t commandID(payload[offset + 2]);
			const size_t messageBytes(size_t(4) + commandLength);
			const size_t paddedBytes((messageBytes + 3) & ~size_t(3));
			if (offset + paddedBytes > payloadBytes)
			{
				truncated = true;
				break;
			}
			if (messages < 3)
			{
				if (messages)
					samples << ";";
				samples << "dst:" << unsigned(destination)
						<< ",cmd:" << unsigned(commandID)
						<< ",len:" << unsigned(commandLength);
				if (commandID == 0 && commandLength >= 4)
				{
					const uint8_t category(payload[offset + 4]);
					const uint8_t parameter(payload[offset + 5]);
					const uint8_t dataType(payload[offset + 6]);
					const uint8_t operation(payload[offset + 7]);
					const uint8_t * data(payload + offset + 8);
					const size_t dataBytes(commandLength - 4);
					const string name(BlackmagicParameterName(category, parameter));
					const string value(BlackmagicDecodedValue(dataType, data, dataBytes));
					samples << ",field:" << name
							<< ",op:" << unsigned(operation)
							<< ",value:" << value;
					if (decoded.size() < 8)
						decoded.push_back(name + "=" + value);
					if (outMetadata)
						ApplyBlackmagicCameraMetadataField(*outMetadata, category, parameter, dataType, data, dataBytes);
				}
			}
			else if (commandID == 0 && commandLength >= 4)
			{
				const uint8_t category(payload[offset + 4]);
				const uint8_t parameter(payload[offset + 5]);
				const uint8_t dataType(payload[offset + 6]);
				const uint8_t * data(payload + offset + 8);
				const size_t dataBytes(commandLength - 4);
				if (outMetadata)
					ApplyBlackmagicCameraMetadataField(*outMetadata, category, parameter, dataType, data, dataBytes);
				if (decoded.size() < 8)
					decoded.push_back(BlackmagicParameterName(category, parameter) + "=" + BlackmagicDecodedValue(dataType, data, dataBytes));
			}
			messages++;
			offset += paddedBytes;
		}

		ostringstream oss;
		oss << "messages:" << DEC(messages) << ",bytes:" << DEC(payloadBytes);
		if (truncated || offset != payloadBytes)
			oss << ",truncated:true";
		if (!decoded.empty())
		{
			if (outMetadata)
				outMetadata->blackmagicCCUDecodedFields = JoinVector(decoded, 300);
			oss << ",decoded:";
			for (size_t ndx(0); ndx < decoded.size(); ndx++)
			{
				if (ndx)
					oss << ";";
				oss << decoded[ndx];
			}
		}
		if (!samples.str().empty())
			oss << "," << samples.str();
		return oss.str();
	}

		static void DecodeKnownFrameStatusPacket (AJAAncillaryData * pPkt, FrameCameraMetadataTelemetry & out)
		{
			if (!pPkt || !pPkt->GetPayloadData())
				return;
		const uint8_t did(pPkt->GetDID());
		const uint8_t sid(pPkt->GetSID());
		const uint8_t * payload(pPkt->GetPayloadData());
		const size_t payloadBytes(pPkt->GetPayloadByteCount());
			if (did == 0x52 && sid == 0x4d && payloadBytes == 0x0b)
			{
				out.recordFlag = ((payload[10] & 0x03) == 0) ? "false" : "true";
				if (out.cameraVendor == "-")
					out.cameraVendor = "Canon/Sony";
				if (out.cameraMetadataConfidence == "-")
					out.cameraMetadataConfidence = "frame_status_0x524d";
				const string summary(string("frame_status_0x524d,record:") + out.recordFlag + ",bytes:" + IntegerValue(payloadBytes));
				out.sonyMetadataSummary = summary;
				out.sonyMetadataPayloadHex = PayloadHex(payload, payloadBytes);
				AppendPacketID(out.sonyMetadataPacketIDs, did, sid);
				out.canonMetadataSummary = summary;
				out.canonMetadataPayloadHex = PayloadHex(payload, payloadBytes);
				AppendPacketID(out.canonMetadataPacketIDs, did, sid);
			}
			else if (did == 0x52 && sid == 0x51 && payloadBytes == 0x08)
			{
				out.recordFlag = ((payload[0] & 0x60) == 0x20) ? "true" : "false";
				if (out.cameraVendor == "-")
					out.cameraVendor = "Canon";
				if (out.cameraMetadataConfidence == "-")
					out.cameraMetadataConfidence = "frame_status_0x5251";
				out.canonMetadataSummary = string("frame_status_0x5251,record:") + out.recordFlag + ",bytes:" + IntegerValue(payloadBytes);
				out.canonMetadataPayloadHex = PayloadHex(payload, payloadBytes);
				AppendPacketID(out.canonMetadataPacketIDs, did, sid);
			}
			else if ((did == 0x43 && sid == 0x05) || (did == 0x51 && sid == 0x02))
			{
				if (out.cameraMetadataConfidence == "-")
					out.cameraMetadataConfidence = "rdd18_camera_params";
				out.rdd18PayloadHex = PayloadHex(payload, payloadBytes);
				AppendPacketID(out.rdd18PacketIDs, did, sid);
			}
			else if (did == 0x44 && (sid == 0x04 || sid == 0x14))
			{
				if (out.cameraMetadataConfidence == "-")
					out.cameraMetadataConfidence = "rp214_klv_metadata";
				out.sdiKLVPayloadHex = PayloadHex(payload, payloadBytes);
				AppendPacketID(out.sdiKLVPacketIDs, did, sid);
			}
			else if (did == 0x51 && sid == 0x52)
			{
				if (out.cameraVendor == "-")
					out.cameraVendor = "Blackmagic";
				if (out.cameraMetadataConfidence == "-")
					out.cameraMetadataConfidence = "blackmagic_embedded_tally_0x5152";
				out.blackmagicTallyPayloadHex = PayloadHex(payload, payloadBytes);
				ApplyBlackmagicTallyPayload(payload, payloadBytes, out);
				AppendPacketID(out.blackmagicTallyPacketIDs, did, sid);
			}
			else if (did == 0x51 && sid == 0x53)
			{
				if (out.cameraVendor == "-")
					out.cameraVendor = "Blackmagic";
				if (out.cameraMetadataConfidence == "-")
					out.cameraMetadataConfidence = "blackmagic_sdi_camera_control_0x5153";
				out.blackmagicCCUPayloadHex = PayloadHex(payload, payloadBytes);
				out.blackmagicCCUSummary = DecodeBlackmagicCCUSummary(payload, payloadBytes, &out);
				AppendPacketID(out.blackmagicCCUPacketIDs, did, sid);
			}
		}

	static FrameCameraMetadataTelemetry ExtractFrameCameraMetadataTelemetry (NTV2FrameData & inFrameData, const bool inUseHDMIAux)
	{
		FrameCameraMetadataTelemetry out;
		out.hasAnc = false;
		out.ancBytes = size_t(inFrameData.fNumAncBytes) + size_t(inFrameData.fNumAnc2Bytes);
			out.ancPackets = 0;
			out.cameraVendor = "-";
			out.cameraModel = "-";
			out.cameraID = "-";
			out.cameraSerial = "-";
			out.cameraFirmware = "-";
			out.lensData = "-";
			out.isoEI = "-";
			out.shutter = "-";
			out.whiteBalance = "-";
		out.tint = "-";
		out.nd = "-";
			out.aperture = "-";
			out.focusDistance = "-";
			out.cameraMediaFilename = "-";
			out.cameraClipName = "-";
			out.reelName = "-";
			out.codec = "-";
			out.recordingFormat = "-";
			out.sensorFPS = "-";
			out.projectFPS = "-";
			out.colorSpace = "-";
			out.gamma = "-";
			out.gamut = "-";
			out.lutName = "-";
			out.lookName = "-";
			out.anamorphicSqueeze = "-";
			out.recordFlag = "-";
			out.cameraTimecode = "-";
			out.cameraMetadataConfidence = "-";
			out.ancHash = "-";
			out.ancPacketIDs = "-";
			out.ancPayloadTypes = "-";
				out.ancTextPreview = "-";
			out.ancRawPrefixHex = "-";
				out.rdd18PayloadHex = "-";
			out.rdd18PacketIDs = "-";
			out.sdiKLVPayloadHex = "-";
			out.sdiKLVPacketIDs = "-";
			out.sdiKLVSummary = "-";
			out.arriMetadataPayloadHex = "-";
			out.arriMetadataPacketIDs = "-";
			out.arriMetadataSummary = "-";
			out.sonyMetadataPayloadHex = "-";
			out.sonyMetadataPacketIDs = "-";
			out.sonyMetadataSummary = "-";
			out.canonMetadataPayloadHex = "-";
			out.canonMetadataPacketIDs = "-";
			out.canonMetadataSummary = "-";
			out.redMetadataPayloadHex = "-";
			out.redMetadataPacketIDs = "-";
			out.redMetadataSummary = "-";
			out.blackmagicTallyPayloadHex = "-";
		out.blackmagicTallyPacketIDs = "-";
		out.blackmagicTallySummary = "-";
		out.blackmagicTallyMonitor = "-";
		out.blackmagicTallyVersion = "-";
		out.blackmagicTallySlaveStates = "-";
		out.blackmagicCCUPayloadHex = "-";
		out.blackmagicCCUPacketIDs = "-";
		out.blackmagicCCUSummary = "-";
		out.blackmagicCCUDecodedFields = "-";
		out.blackmagicVideoMode = "-";
		out.blackmagicRecordingFormat = "-";
		out.blackmagicCodec = "-";
		out.blackmagicTransportMode = "-";
		out.blackmagicDynamicRange = "-";
		out.blackmagicDisplayLUT = "-";
		out.blackmagicTimecodeSource = "-";
		out.blackmagicReference = "-";
		out.blackmagicAudio = "-";
		out.blackmagicOutput = "-";
		out.blackmagicDisplay = "-";
		out.blackmagicColorCorrection = "-";
		out.blackmagicConfiguration = "-";
		out.blackmagicPTZ = "-";

		if (!out.ancBytes)
			return out;

		out.hasAnc = true;
		uint32_t hash(2166136261u);
			if (inFrameData.AncBuffer() && inFrameData.fNumAncBytes)
			{
				hash = UpdateFNV1a(hash, inFrameData.AncBuffer().GetHostPointer(), inFrameData.fNumAncBytes);
				out.ancRawPrefixHex = PayloadHex(reinterpret_cast<const uint8_t *>(inFrameData.AncBuffer().GetHostPointer()), inFrameData.fNumAncBytes, 128);
			}
			if (inFrameData.AncBuffer2() && inFrameData.fNumAnc2Bytes)
				hash = UpdateFNV1a(hash, inFrameData.AncBuffer2().GetHostPointer(), inFrameData.fNumAnc2Bytes);
		out.ancHash = HexValue(hash, 8);

			string payloadText;
			set<string> packetIDs;
			set<string> packetTypes;
			vector<KLVPacketFragment> klvFragments;
			NTV2Buffer f1(inFrameData.AncBuffer().GetHostPointer(), inFrameData.fNumAncBytes);
		NTV2Buffer f2(inFrameData.AncBuffer2().GetHostPointer(), inFrameData.fNumAnc2Bytes);
			AJAAncillaryList packets;
			const AJAStatus packetStatus(inUseHDMIAux
				? AJAAncillaryList::SetFromDeviceAuxBuffers(f1, f2, packets)
				: AJAAncillaryList::SetFromDeviceAncBuffers(f1, f2, packets));
			if (AJA_SUCCESS(packetStatus))
			{
				packets.ParseAllAncillaryData();
				out.ancPackets = packets.CountAncillaryData();
				for (uint32_t ndx(0); ndx < packets.CountAncillaryData(); ndx++)
				{
					AJAAncillaryData * pPkt(packets.GetAncillaryDataAtIndex(ndx));
					if (!pPkt)
						continue;
					if (inUseHDMIAux)
					{
						const string auxID(HexValue(pPkt->GetAuxType()));
						const string auxHeader(auxID + "/" + HexValue(pPkt->GetAuxHB1()) + "/" + HexValue(pPkt->GetAuxHB2()));
						packetIDs.insert(auxHeader);
						packetTypes.insert(auxID + ":" + AJAAncillaryData::AuxPacketTypeToString(pPkt->GetAuxType()));
						AppendPrintableRuns(pPkt->GetPayloadData(), pPkt->GetPayloadByteCount(), payloadText);
						continue;
					}
						const string didsid(HexValue(pPkt->GetDID()) + "/" + HexValue(pPkt->GetSID()));
							packetIDs.insert(didsid);
							packetTypes.insert(didsid + ":" + KnownAncillaryTypeName(pPkt->GetDID(), pPkt->GetSID()));
						DecodeKnownFrameStatusPacket(pPkt, out);
						if (pPkt->GetDID() == 0x44 && (pPkt->GetSID() == 0x04 || pPkt->GetSID() == 0x14) && pPkt->GetPayloadData() && pPkt->GetPayloadByteCount())
						{
							KLVPacketFragment fragment;
							fragment.did = pPkt->GetDID();
							fragment.sid = pPkt->GetSID();
							fragment.messageID = pPkt->GetPayloadData()[0];
							fragment.segmentIndex = ReadKLVSegmentIndex(pPkt->GetPayloadData(), pPkt->GetPayloadByteCount());
							fragment.payload.assign(pPkt->GetPayloadData(), pPkt->GetPayloadData() + pPkt->GetPayloadByteCount());
							klvFragments.push_back(fragment);
						}
						AppendPrintableRuns(pPkt->GetPayloadData(), pPkt->GetPayloadByteCount(), payloadText);
					}
			}

			DecodeRP214KLVSummary(klvFragments, out);

		if (payloadText.empty())
		{
			AppendPrintableRuns(reinterpret_cast<const uint8_t *>(inFrameData.AncBuffer().GetHostPointer()), inFrameData.fNumAncBytes, payloadText);
			AppendPrintableRuns(reinterpret_cast<const uint8_t *>(inFrameData.AncBuffer2().GetHostPointer()), inFrameData.fNumAnc2Bytes, payloadText);
		}

		out.ancPacketIDs = JoinSet(packetIDs);
		out.ancPayloadTypes = JoinSet(packetTypes);
		out.ancTextPreview = payloadText.empty() ? "-" : SanitizedMetadataValue(payloadText.substr(0, min<size_t>(payloadText.size(), 160)));

			if (ContainsCI(payloadText, "arri") || ContainsCI(payloadText, "alexa") || ContainsCI(payloadText, "amira"))
			{
				out.cameraVendor = "ARRI";
				if (out.cameraModel == "-")
					out.cameraModel = MatchFirstMarker(payloadText, vector<string>{"ALEXA 35", "ALEXA Mini LF", "ALEXA LF", "ALEXA Mini", "ALEXA", "AMIRA"});
				if (out.cameraMetadataConfidence == "-")
					out.cameraMetadataConfidence = "vendor_marker";
			}
			else if (ContainsCI(payloadText, "sony") || ContainsCI(payloadText, "venice") || ContainsCI(payloadText, "cinealta"))
			{
				out.cameraVendor = "Sony";
				if (out.cameraModel == "-")
					out.cameraModel = MatchFirstMarker(payloadText, vector<string>{"VENICE 2", "VENICE", "BURANO", "CineAlta", "FX9", "FX6", "F55", "F5", "F3"});
				if (out.cameraMetadataConfidence == "-")
					out.cameraMetadataConfidence = "vendor_marker";
			}
			else if (ContainsCI(payloadText, "canon") || ContainsCI(payloadText, "eos") || ContainsCI(payloadText, "c300") || ContainsCI(payloadText, "c500"))
			{
				out.cameraVendor = "Canon";
				if (out.cameraModel == "-")
					out.cameraModel = MatchFirstMarker(payloadText, vector<string>{"C500 Mark II", "C500", "C400", "C300 Mark III", "C300", "C70", "EOS C"});
				if (out.cameraMetadataConfidence == "-")
					out.cameraMetadataConfidence = "vendor_marker";
			}
			else if (ContainsCI(payloadText, "blackmagic") || ContainsCI(payloadText, "bmd") || ContainsCI(payloadText, "ursa"))
			{
				out.cameraVendor = "Blackmagic";
				if (out.cameraModel == "-")
					out.cameraModel = MatchFirstMarker(payloadText, vector<string>{"URSA Cine", "URSA Mini Pro", "URSA Broadcast", "Pocket Cinema", "Blackmagic"});
				if (out.cameraMetadataConfidence == "-")
					out.cameraMetadataConfidence = "vendor_marker";
			}
			else if (ContainsCI(payloadText, "red") || ContainsCI(payloadText, "komodo") || ContainsCI(payloadText, "v-raptor") || ContainsCI(payloadText, "dsmc"))
			{
				out.cameraVendor = "RED";
				if (out.cameraModel == "-")
					out.cameraModel = MatchFirstMarker(payloadText, vector<string>{"V-RAPTOR XL", "V-RAPTOR", "KOMODO-X", "KOMODO", "DSMC2", "DSMC", "EPIC", "SCARLET"});
				if (out.cameraMetadataConfidence == "-")
					out.cameraMetadataConfidence = "vendor_marker";
			}

				auto fillIfMissing = [](string & target, const string & value) {
					if (target == "-" && TextMetadataValuePresent(value))
						target = value;
				};
				if (out.cameraModel == "-")
				{
					const string cameraModel(ExtractAfterLabel(payloadText, vector<string>{"camera model", "model", "cam"}));
					if (TextMetadataValuePresent(cameraModel))
						out.cameraModel = cameraModel;
				}
			fillIfMissing(out.cameraID, ExtractAfterLabel(payloadText, vector<string>{"camera id", "camera letter", "cam id"}));
			fillIfMissing(out.cameraSerial, ExtractAfterLabel(payloadText, vector<string>{"serial", "camera serial", "body serial"}));
			fillIfMissing(out.cameraFirmware, ExtractAfterLabel(payloadText, vector<string>{"firmware", "sup", "software version"}));
			fillIfMissing(out.lensData, ExtractAfterLabel(payloadText, vector<string>{"lens data", "lens", "lensed"}));
			fillIfMissing(out.isoEI, ExtractAfterLabel(payloadText, vector<string>{"exposure index", "iso", "ei", "gain"}));
			fillIfMissing(out.shutter, ExtractAfterLabel(payloadText, vector<string>{"shutter angle", "shutter speed", "shutter"}));
			fillIfMissing(out.whiteBalance, ExtractAfterLabel(payloadText, vector<string>{"white balance", "whitebalance", "wb", "kelvin"}));
			fillIfMissing(out.tint, ExtractAfterLabel(payloadText, vector<string>{"tint", "cc"}));
			fillIfMissing(out.nd, ExtractAfterLabel(payloadText, vector<string>{"nd filter", "nd"}));
			fillIfMissing(out.aperture, ExtractAfterLabel(payloadText, vector<string>{"aperture", "iris", "t-stop", "tstop"}));
			fillIfMissing(out.focusDistance, ExtractAfterLabel(payloadText, vector<string>{"focus distance", "focus", "distance"}));
			fillIfMissing(out.cameraClipName, ExtractAfterLabel(payloadText, vector<string>{"camera clip name", "clip filename", "clip name"}));
			fillIfMissing(out.reelName, ExtractAfterLabel(payloadText, vector<string>{"reel name", "roll", "reel"}));
			fillIfMissing(out.cameraMediaFilename, ExtractAfterLabel(payloadText, vector<string>{"camera media filename", "media filename", "filename", "clip filename", "clip name", "reel"}));
			fillIfMissing(out.codec, ExtractAfterLabel(payloadText, vector<string>{"codec", "compression"}));
			fillIfMissing(out.recordingFormat, ExtractAfterLabel(payloadText, vector<string>{"recording format", "record format", "format"}));
			fillIfMissing(out.sensorFPS, ExtractAfterLabel(payloadText, vector<string>{"sensor fps", "capture fps", "sensor frame rate"}));
			fillIfMissing(out.projectFPS, ExtractAfterLabel(payloadText, vector<string>{"project fps", "project frame rate", "time base"}));
			fillIfMissing(out.colorSpace, ExtractAfterLabel(payloadText, vector<string>{"color space", "colorspace"}));
			fillIfMissing(out.gamma, ExtractAfterLabel(payloadText, vector<string>{"gamma", "log curve", "curve"}));
			fillIfMissing(out.gamut, ExtractAfterLabel(payloadText, vector<string>{"gamut"}));
			fillIfMissing(out.lutName, ExtractAfterLabel(payloadText, vector<string>{"lut", "lut name", "display lut"}));
			fillIfMissing(out.lookName, ExtractAfterLabel(payloadText, vector<string>{"look", "look name", "look file"}));
			fillIfMissing(out.anamorphicSqueeze, ExtractAfterLabel(payloadText, vector<string>{"anamorphic squeeze", "squeeze", "pixel aspect"}));
			if (out.cameraMediaFilename == "-")
				out.cameraMediaFilename = out.cameraClipName;
			if (out.cameraVendor == "ARRI" && out.arriMetadataSummary == "-")
			{
				out.arriMetadataSummary = "vendor_marker";
				if (out.sdiKLVPacketIDs != "-")
					out.arriMetadataPacketIDs = out.sdiKLVPacketIDs;
				if (out.sdiKLVPayloadHex != "-")
					out.arriMetadataPayloadHex = out.sdiKLVPayloadHex;
			}
			else if (out.cameraVendor == "Sony" && out.sonyMetadataSummary == "-")
				out.sonyMetadataSummary = "vendor_marker";
			else if (out.cameraVendor == "Canon" && out.canonMetadataSummary == "-")
				out.canonMetadataSummary = "vendor_marker";
			else if (out.cameraVendor == "RED" && out.redMetadataSummary == "-")
			{
				out.redMetadataSummary = (out.ancPacketIDs.find("0x60/0x60") != string::npos)
					? "vendor_marker,smpte12m_red_hanc_carrier_present"
					: "vendor_marker";
				if (out.ancPacketIDs.find("0x60/0x60") != string::npos)
					out.redMetadataPacketIDs = "0x60/0x60";
			}
			if (out.recordFlag == "-")
			{
			const string lowerPayload(LowercaseString(payloadText));
			if (lowerPayload.find("record=1") != string::npos || lowerPayload.find("rec=1") != string::npos || lowerPayload.find("recording") != string::npos)
				out.recordFlag = "true";
			else if (lowerPayload.find("record=0") != string::npos || lowerPayload.find("rec=0") != string::npos || lowerPayload.find("standby") != string::npos)
				out.recordFlag = "false";
		}

		return out;
	}

	static FrameTimecodeTelemetry ExtractFrameTimecodeTelemetry (NTV2FrameData & inFrameData, const NTV2VideoFormat inVideoFormat)
	{
		FrameTimecodeTelemetry out;
		out.valid = false;
		out.source = -1;
		out.timecode = "-";
		out.userBits = "-";
		out.raw = NTV2_RP188();

		if (inFrameData.fTimecodes.empty())
			return out;

		NTV2TimeCodesConstIter chosen (inFrameData.fTimecodes.end());
		for (NTV2TimeCodesConstIter it (inFrameData.fTimecodes.begin()); it != inFrameData.fTimecodes.end(); ++it)
			if (inFrameData.HasValidTimecode(it->first))
				{chosen = it; break;}
		if (chosen == inFrameData.fTimecodes.end())
			chosen = inFrameData.fTimecodes.begin();

		const NTV2FrameRate ntv2FrameRate (::GetNTV2FrameRateFromVideoFormat(inVideoFormat));
		const TimecodeFormat tcFormat (CNTV2DemoCommon::NTV2FrameRate2TimecodeFormat(ntv2FrameRate));
		CRP188 rp188 (chosen->second, tcFormat);
		string timecode, userBits;
		rp188.GetRP188Str(timecode);
		rp188.GetRP188UserBitsStr(userBits);

		out.valid = inFrameData.HasValidTimecode(chosen->first);
		out.source = int(chosen->first);
		out.timecode = SanitizedMetadataValue(timecode);
		out.userBits = SanitizedMetadataValue(userBits);
		out.raw = chosen->second;
		return out;
	}

	static void PutCString (uint8_t * pDst, const size_t maxBytes, const string & value)
	{
		if (!pDst || !maxBytes)
			return;
		memset(pDst, 0, maxBytes);
		const size_t count = min(maxBytes, value.size());
		if (count)
			memcpy(pDst, value.data(), count);
	}
	}

	static AudioLTCProbeResult ProbeAudioLTCCandidates (const int32_t * samples,
														const size_t sampleFrames,
														const size_t numChannels,
														const size_t channelCount,
														const double sampleRateHz,
														const vector<double> & rmsDbByChannel)
	{
		AudioLTCProbeResult out;
		out.candidateChannels = "-";
		out.zeroCrossHz = "-";
		out.rmsDb = "-";
		out.confidence = "-";

		if (!samples || sampleFrames < 256 || !numChannels || !channelCount || sampleRateHz <= 0.0)
			return out;

		vector<double> zeroCrossHz(channelCount, 0.0);
		vector<size_t> candidates;
		for (size_t channelIndex = 0; channelIndex < channelCount; channelIndex++)
		{
			size_t transitions(0);
			int previousSign(0);
			for (size_t frameIndex = 0; frameIndex < sampleFrames; frameIndex++)
			{
				const int32_t rawSample = samples[frameIndex * numChannels + channelIndex];
				const int sign = rawSample > 0 ? 1 : (rawSample < 0 ? -1 : 0);
				if (!sign)
					continue;
				if (previousSign && sign != previousSign)
					transitions++;
				previousSign = sign;
			}
			const double hz = (double(transitions) * sampleRateHz) / double(sampleFrames);
			zeroCrossHz[channelIndex] = hz;
			const double rmsDb = channelIndex < rmsDbByChannel.size() ? rmsDbByChannel[channelIndex] : -90.0;
			// LTC is bi-phase mark encoded audio. This is intentionally a probe, not a decoder:
			// it only flags strong square-wave-like embedded audio that is plausible LTC.
			if (rmsDb > -70.0 && hz >= 1200.0 && hz <= 7000.0)
				candidates.push_back(channelIndex);
		}

		ostringstream zeroStream;
		ostringstream rmsStream;
		zeroStream << fixed << setprecision(0);
		rmsStream << fixed << setprecision(1);
		for (size_t channelIndex = 0; channelIndex < channelCount; channelIndex++)
		{
			if (channelIndex)
			{
				zeroStream << ",";
				rmsStream << ",";
			}
			zeroStream << zeroCrossHz[channelIndex];
			rmsStream << (channelIndex < rmsDbByChannel.size() ? rmsDbByChannel[channelIndex] : -90.0);
		}
		out.zeroCrossHz = zeroStream.str();
		out.rmsDb = rmsStream.str();

		if (!candidates.empty())
		{
			ostringstream channelStream;
			for (size_t i = 0; i < candidates.size(); i++)
			{
				if (i)
					channelStream << ",";
				channelStream << DEC(candidates[i] + 1);
			}
			out.candidateChannels = channelStream.str();
			out.confidence = "candidate";
		}
		else
			out.confidence = "none";

		return out;
	}


	//////////////////////////////////////////////////////////////////////////////////////	NTV2Capture IMPLEMENTATION

NTV2Capture::NTV2Capture (const CaptureConfig & inConfig)
	:	mConsumerThread		(AJAThread()),
		mProducerThread		(AJAThread()),
		mDeviceID			(DEVICE_ID_NOTFOUND),
		mConfig				(inConfig),
		mVideoFormat		(NTV2_FORMAT_UNKNOWN),
		mSavedTaskMode		(NTV2_DISABLE_TASKS),
		mAudioSystem		(inConfig.fWithAudio ? NTV2_AUDIOSYSTEM_1 : NTV2_AUDIOSYSTEM_INVALID),
		mHostBuffers		(),
		mAVCircularBuffer	(),
		mGlobalQuit			(false),
			mUseLevelBTSIRouting(false),
			mSnapshotCaptured	(false),
			mSnapshotPath		(),
			mRawStreamEnabled	(false),
			mRawStreamFrameCount(0),
			mRawStreamEveryN	(1),
			mRawStreamSeenCount	(0),
			mRawStreamDownsample(1),
			mRawScratch		(),
			mFrameMapEnabled	(false),
			mFrameMapPath		(),
			mFrameMapMaxBytes	(0),
				mFrameMapFD			(-1),
				mFrameMapPtr		(AJA_NULL),
				mFrameMapSize		(0),
				mFrameMapSeq		(0),
				mFrameMapBank		(0),
				mLastTimecodeLogMs	(0),
				mLastTimecodeTelemetry(),
				mLastAudioLogMs		(0),
				mLastAudioTelemetry(),
				mLastCameraMetadataLogMs	(0),
				mLastCameraMetadataTelemetry()
{
}	//	constructor


void NTV2Capture::EnableSnapshot (const std::string & inFilePath)
{
	mSnapshotPath = inFilePath;
}


bool NTV2Capture::SnapshotCaptured (void) const
{
	return mSnapshotCaptured;
}


void NTV2Capture::EnableRawStream (void)
{
	mRawStreamEnabled = true;
}


void NTV2Capture::EnableFrameMap (const std::string & inFilePath, const ULWord inMaxBytes)
{
	mFrameMapEnabled = !inFilePath.empty();
	mFrameMapPath = inFilePath;
	mFrameMapMaxBytes = inMaxBytes;
}


bool NTV2Capture::RawStreamEnabled (void) const
{
	return mRawStreamEnabled;
}


bool NTV2Capture::FrameMapEnabled (void) const
{
	return mFrameMapEnabled;
}


void NTV2Capture::SetRawStreamFrameDecimation (const ULWord inEveryN)
{
	mRawStreamEveryN = inEveryN > 0 ? inEveryN : 1;
}


void NTV2Capture::SetRawStreamDownsample (const ULWord inFactor)
{
	mRawStreamDownsample = inFactor > 0 ? inFactor : 1;
}


NTV2Capture::~NTV2Capture ()
{
	//	Stop my capture and consumer threads, then destroy them...
	Quit();
	TeardownFrameMap();

	//	Unsubscribe from VBI events...
	mDevice.UnsubscribeInputVerticalEvent(mConfig.fInputChannel);
	mDevice.UnsubscribeOutputVerticalEvent(NTV2_CHANNEL1);

}	//	destructor


void NTV2Capture::Quit (void)
{
	//	Set the global 'quit' flag, and wait for the threads to go inactive...
	mGlobalQuit = true;

	while (mConsumerThread.Active())
		AJATime::Sleep(10);

	while (mProducerThread.Active())
		AJATime::Sleep(10);

	//	Restore some of the device's former state...
	if (!mConfig.fDoMultiFormat)
	{
		mDevice.ReleaseStreamForApplication (kDemoAppSignature, int32_t(AJAProcess::GetPid()));
		mDevice.SetTaskMode(mSavedTaskMode);	//	Restore prior task mode
	}

}	//	Quit


bool NTV2Capture::SetupFrameMap (void)
{
	if (!mFrameMapEnabled || mFrameMapPath.empty())
		return false;
	if (mFrameMapPtr && mFrameMapFD >= 0)
		return true;

	const ULWord width (mFormatDesc.GetRasterWidth());
	const ULWord height (mFormatDesc.GetVisibleRasterHeight());
	const ULWord scale = mRawStreamDownsample > 0 ? mRawStreamDownsample : 1;
	const ULWord outWidth = width / scale;
	const ULWord outHeight = height / scale;
	const ULWord tightBytesPerRow (outWidth * 4);
	const ULWord payloadBytes (tightBytesPerRow * outHeight);
	const size_t headerBytes (64);
	// Always allocate two payload banks to avoid reader/writer tearing at high frame rates.
	size_t mapSize = headerBytes + size_t(payloadBytes) * 2;
	if (mFrameMapMaxBytes > 0 && size_t(mFrameMapMaxBytes) > mapSize)
		mapSize = size_t(mFrameMapMaxBytes);
	if (mapSize < (headerBytes + 4096))
		mapSize = headerBytes + 4096;

	// Do not O_TRUNC here: readers may have an existing mapping, and truncation can SIGBUS them.
	mFrameMapFD = ::open(mFrameMapPath.c_str(), O_RDWR | O_CREAT, 0600);
	if (mFrameMapFD < 0)
	{
		cerr << "## ERROR: frame map open failed '" << mFrameMapPath << "'" << endl;
		return false;
	}
	struct stat st;
	if (::fstat(mFrameMapFD, &st) == 0)
	{
		const size_t existingSize = st.st_size > 0 ? size_t(st.st_size) : 0;
		if (existingSize > mapSize)
			mapSize = existingSize;	// Never shrink an existing map while readers may be active.
	}
	if (::ftruncate(mFrameMapFD, off_t(mapSize)) != 0)
	{
		cerr << "## ERROR: frame map ftruncate failed '" << mFrameMapPath << "'" << endl;
		::close(mFrameMapFD);
		mFrameMapFD = -1;
		return false;
	}
	void * mapped = ::mmap(AJA_NULL, mapSize, PROT_READ | PROT_WRITE, MAP_SHARED, mFrameMapFD, 0);
	if (mapped == MAP_FAILED)
	{
		cerr << "## ERROR: frame map mmap failed '" << mFrameMapPath << "'" << endl;
		::close(mFrameMapFD);
		mFrameMapFD = -1;
		return false;
	}
	mFrameMapPtr = reinterpret_cast<uint8_t *>(mapped);
	mFrameMapSize = mapSize;
	mFrameMapSeq = 0;
	mFrameMapBank = 0;
	::memset(mFrameMapPtr, 0, mFrameMapSize);
	::memcpy(mFrameMapPtr, "RMAP", 4);
	mFrameMapPtr[4] = 2;
	mFrameMapPtr[5] = 0;
	mFrameMapPtr[6] = 0;
	mFrameMapPtr[7] = 0;
	for (int i = 0; i < 8; ++i)
		mFrameMapPtr[24 + i] = 0;
	for (int i = 0; i < 4; ++i)
	{
		mFrameMapPtr[28 + i] = 0; // active payload bank
		mFrameMapPtr[32 + i] = 0; // payload stride (0 => single buffer)
	}
	return true;
}


void NTV2Capture::TeardownFrameMap (void)
{
	if (mFrameMapPtr && mFrameMapSize > 0)
	{
		::munmap(mFrameMapPtr, mFrameMapSize);
		mFrameMapPtr = AJA_NULL;
		mFrameMapSize = 0;
	}
	if (mFrameMapFD >= 0)
	{
		::close(mFrameMapFD);
		mFrameMapFD = -1;
	}
}


AJAStatus NTV2Capture::Init (void)
{
	AJAStatus status (AJA_STATUS_SUCCESS);

	//	Open the device...
	if (!CNTV2DeviceScanner::GetFirstDeviceFromArgument (mConfig.fDeviceSpec, mDevice))
		{cerr << "## ERROR:  Device '" << mConfig.fDeviceSpec << "' not found" << endl;  return AJA_STATUS_OPEN;}

	if (!mDevice.IsDeviceReady())
		{cerr << "## ERROR:  '" << mDevice.GetDisplayName() << "' not ready" << endl;  return AJA_STATUS_INITIALIZE;}

	mDeviceID = mDevice.GetDeviceID();						//	Keep the device ID handy, as it's used frequently
	const bool isKonaHDMI (mDevice.features().GetNumHDMIVideoInputs() > 1);
	const NTV2IOKinds inputType (isKonaHDMI ? NTV2_IOKINDS_HDMI : NTV2_IOKINDS_SDI);
	if (!mDevice.features().CanDoCapture())
		{cerr << "## ERROR:  '" << mDevice.GetDisplayName() << "' is playback-only" << endl;  return AJA_STATUS_FEATURE;}

	if (!mDevice.features().CanDoFrameBufferFormat(mConfig.fPixelFormat))
	{	cerr	<< "## ERROR:  '" << mDevice.GetDisplayName() << "' doesn't support '"
				<< ::NTV2FrameBufferFormatToString(mConfig.fPixelFormat, true) << "' ("
				<< ::NTV2FrameBufferFormatToString(mConfig.fPixelFormat, false) << ", " << DEC(mConfig.fPixelFormat) << ")" << endl;
		return AJA_STATUS_UNSUPPORTED;
	}

	ULWord	appSignature	(0);
	int32_t	appPID			(0);
	mDevice.GetStreamingApplication (appSignature, appPID);	//	Who currently "owns" the device?
	mDevice.GetTaskMode(mSavedTaskMode);					//	Save the current device state
	if (!mConfig.fDoMultiFormat)
	{
		if (!mDevice.AcquireStreamForApplication (kDemoAppSignature, int32_t(AJAProcess::GetPid())))
		{
			cerr << "## ERROR:  Unable to acquire '" << mDevice.GetDisplayName() << "' because another app (pid " << appPID << ") owns it" << endl;
			return AJA_STATUS_BUSY;	//	Another app is using the device
		}
	}
	mDevice.SetTaskMode(NTV2_OEM_TASKS);		//	Prevent interference from AJA retail services

	if (mDevice.features().CanDoMultiFormat())
		mDevice.SetMultiFormatMode(mConfig.fDoMultiFormat);

	//	This demo permits input source and channel to be specified independently.
	if (!NTV2_IS_VALID_CHANNEL(mConfig.fInputChannel)  &&  !NTV2_IS_VALID_INPUT_SOURCE(mConfig.fInputSource))
		mConfig.fInputChannel = NTV2_CHANNEL1;	//	Neither -i or -c specified:  default to Ch1
	if (!NTV2_IS_VALID_CHANNEL(mConfig.fInputChannel)  &&  NTV2_IS_VALID_INPUT_SOURCE(mConfig.fInputSource))
		mConfig.fInputChannel = ::NTV2InputSourceToChannel(mConfig.fInputSource);	//	Only -i specified:  use that channel
	else if (NTV2_IS_VALID_CHANNEL(mConfig.fInputChannel)  &&  !NTV2_IS_VALID_INPUT_SOURCE(mConfig.fInputSource))
		mConfig.fInputSource = ::NTV2ChannelToInputSource ( mConfig.fInputChannel, inputType);	//	Only -c specified: use that input source
	//	On KonaHDMI, map specified SDI input to equivalent HDMI input...
	if (isKonaHDMI  &&  NTV2_INPUT_SOURCE_IS_SDI(mConfig.fInputSource))
		mConfig.fInputSource = ::NTV2ChannelToInputSource(::NTV2InputSourceToChannel(mConfig.fInputSource), inputType);
	if (!mDevice.features().CanDoInputSource(mConfig.fInputSource))
	{
		cerr	<< "## ERROR:  No such input '" << ::NTV2InputSourceToString(mConfig.fInputSource, /*compact?*/true)
				<< "' on '" << mDevice.GetDisplayName() << "'" << endl;
		return AJA_STATUS_UNSUPPORTED;
	}
	if (mConfig.fWithAnc  &&  !mDevice.features().CanDoCustomAnc())
		{cerr << "## ERROR: Anc capture requested, but '" << mDevice.GetDisplayName() << "' has no anc extractors";  return AJA_STATUS_UNSUPPORTED;}

	//	Set up the video and audio...
	status = SetupVideo();
	if (AJA_FAILURE(status))
		return status;

	if (mConfig.fWithAudio)
		status = SetupAudio();
	if (AJA_FAILURE(status))
		return status;

	//	Set up the circular buffers, the device signal routing, and both playout and capture AutoCirculate...
	SetupHostBuffers();
	if (!RouteInputSignal())
		return AJA_STATUS_FAIL;
	if (mFrameMapEnabled && !SetupFrameMap())
		return AJA_STATUS_FAIL;

	#if defined(_DEBUG)
		cerr << mConfig;
		if (mDevice.IsRemote())
			cerr	<< "Device Description:  " << mDevice.GetDescription() << endl;
		cerr << endl;
	#endif	//	defined(_DEBUG)
	return AJA_STATUS_SUCCESS;

}	//	Init


AJAStatus NTV2Capture::SetupVideo (void)
{
	mUseLevelBTSIRouting = false;
	bool forceLevelBTsiRouting(false);
	if (const char * pForceTsi = ::getenv("AJA_LIVE_CAPTURE_FORCE_TSI_LEVELB"))
	{
		const string flag(pForceTsi);
		forceLevelBTsiRouting = (flag == "1" || flag == "true" || flag == "TRUE" || flag == "yes" || flag == "YES");
	}

	//	Enable the FrameStore we intend to use...
	mDevice.EnableChannel(mConfig.fInputChannel);

	//	Enable and subscribe to VBIs (critical on Windows)...
	mDevice.EnableInputInterrupt(mConfig.fInputChannel);
	mDevice.SubscribeInputVerticalEvent(mConfig.fInputChannel);
	mDevice.SubscribeOutputVerticalEvent(NTV2_CHANNEL1);

	//	If the device supports bi-directional SDI and the requested input is SDI,
	//	ensure the SDI connector(s) are configured to receive...
	if (mDevice.features().HasBiDirectionalSDI() && NTV2_INPUT_SOURCE_IS_SDI(mConfig.fInputSource))
	{
		mDevice.SetSDITransmitEnable (mConfig.fInputChannel, false);	//	Set SDI connector(s) to receive
		mDevice.WaitForOutputVerticalInterrupt (NTV2_CHANNEL1, 10);		//	Wait 10 VBIs to allow reciever to lock
	}
	bool sdiLevelBToAEnabled(false);
	bool sdiIs3G(false), sdiIs3Gb(false);
	if (NTV2_INPUT_SOURCE_IS_SDI(mConfig.fInputSource))
	{
		// For 3G SDI inputs, ensure level-B streams are converted to level-A for stable format detection/capture.
		const UWord sdiInputSpigot (UWord(::GetIndexForNTV2InputSource(mConfig.fInputSource)));
		NTV2ChannelSet inSet;
		inSet.insert(mConfig.fInputChannel);
		const bool levelConvBySpigot (mDevice.SetSDIInLevelBtoLevelAConversion(sdiInputSpigot, true));
		const bool levelConvBySet (mDevice.SetSDIInLevelBtoLevelAConversion(inSet, true));
		const bool levelConvOK (levelConvBySpigot || levelConvBySet);
		bool levelConvReadback(false);
		(void)mDevice.GetSDIInLevelBtoLevelAConversion(sdiInputSpigot, levelConvReadback);
		sdiLevelBToAEnabled = levelConvReadback || levelConvOK;
		(void)mDevice.GetSDIInput3GPresent(sdiIs3G, mConfig.fInputChannel);
		(void)mDevice.GetSDIInput3GbPresent(sdiIs3Gb, mConfig.fInputChannel);
		cerr << "AJA_LIVE_CAPTURE_SDI_MODE spigot=" << DEC(sdiInputSpigot)
			 << " levelBToA=" << (sdiLevelBToAEnabled ? 1 : 0)
			 << " levelBToABySpigot=" << (levelConvBySpigot ? 1 : 0)
			 << " levelBToABySet=" << (levelConvBySet ? 1 : 0)
			 << " is3G=" << (sdiIs3G ? 1 : 0)
			 << " is3Gb=" << (sdiIs3Gb ? 1 : 0)
			 << endl;
	}

	//	Determine the input video signal format...
	mVideoFormat = mDevice.GetInputVideoFormat(mConfig.fInputSource);
	if (NTV2_INPUT_SOURCE_IS_SDI(mConfig.fInputSource) && sdiIs3Gb && sdiLevelBToAEnabled)
	{
		const NTV2VideoFormat progressiveFormat (mDevice.GetSDIInputVideoFormat(mConfig.fInputChannel, true));
		if (progressiveFormat != NTV2_FORMAT_UNKNOWN && mDevice.features().CanDoVideoFormat(progressiveFormat))
		{
			if (progressiveFormat != mVideoFormat)
			{
				cerr << "AJA_LIVE_CAPTURE_FORMAT_OVERRIDE base="
					 << CNTV2DemoCommon::StripFormatString(::NTV2VideoFormatToString(mVideoFormat, true))
					 << " progressive="
					 << CNTV2DemoCommon::StripFormatString(::NTV2VideoFormatToString(progressiveFormat, true))
					 << endl;
			}
			mVideoFormat = progressiveFormat;
		}
	}
	if (NTV2_INPUT_SOURCE_IS_SDI(mConfig.fInputSource) && NTV2_VIDEO_FORMAT_IS_B(mVideoFormat))
	{
		const NTV2VideoFormat levelAFormat (LevelBToLevelAFormat(mVideoFormat));
		if (levelAFormat != mVideoFormat && mDevice.features().CanDoVideoFormat(levelAFormat))
		{
			cerr << "AJA_LIVE_CAPTURE_FORMAT_OVERRIDE base="
				 << CNTV2DemoCommon::StripFormatString(::NTV2VideoFormatToString(mVideoFormat, true))
				 << " progressive="
				 << CNTV2DemoCommon::StripFormatString(::NTV2VideoFormatToString(levelAFormat, true))
				 << endl;
			mVideoFormat = levelAFormat;
		}
		// Default to single-link after Level-B->A conversion.
		// Some devices expose DS2 in ways that can corrupt HD Level-B ingest if routed through TSI.
		mUseLevelBTSIRouting = forceLevelBTsiRouting && mDevice.features().CanDo425Mux();
	}
	if (mVideoFormat == NTV2_FORMAT_UNKNOWN)
		{cerr << "## ERROR:  No input signal or unknown format" << endl;  return AJA_STATUS_NOINPUT;}
	if (!mDevice.features().CanDoVideoFormat(mVideoFormat))
	{
		cerr << "## ERROR:  '" << mDevice.GetDisplayName() << "' cannot handle " << ::NTV2VideoFormatToString(mVideoFormat) << endl;
		return AJA_STATUS_UNSUPPORTED;	//	Device can't handle this format
	}
	if (NTV2_IS_4K_VIDEO_FORMAT(mVideoFormat) || NTV2_IS_QUAD_QUAD_FORMAT(mVideoFormat))
	{
		cerr << "## ERROR:  'ntv2capture' cannot handle " << ::NTV2VideoFormatToString(mVideoFormat)
            << " input signal -- try 'ntv2capture4k' or 'ntv2capture8k' demo" << endl;
		return AJA_STATUS_UNSUPPORTED;	//	This demo can't handle this format
	}
	CAPNOTE(::NTV2VideoFormatToString(mVideoFormat) << " detected on " << ::NTV2InputSourceToString(mConfig.fInputSource,true) << " on " << mDevice.GetDisplayName());
	mFormatDesc = NTV2FormatDescriptor(mVideoFormat, mConfig.fPixelFormat);
	{
		const std::string compactFormat (CNTV2DemoCommon::StripFormatString(::NTV2VideoFormatToString(mVideoFormat, true)));
		const NTV2FrameRate fr (::GetNTV2FrameRateFromVideoFormat(mVideoFormat));
		const double fps (::GetFramesPerSecond(fr));
		const bool isPSF (::IsPSF(mVideoFormat));
		const NTV2Standard standard (::GetNTV2StandardFromVideoFormat(mVideoFormat));
		const bool isProgressive (NTV2_IS_PROGRESSIVE_STANDARD(standard) && !isPSF);
		const bool isInterlaced (!isProgressive && !isPSF);
		cerr << "AJA_LIVE_CAPTURE_SIGNAL"
			 << " format=" << compactFormat
			 << " fps=" << fps
			 << " progressive=" << (isProgressive ? 1 : 0)
			 << " interlaced=" << (isInterlaced ? 1 : 0)
			 << " psf=" << (isPSF ? 1 : 0)
			 << endl;
	}

	//	Setting SDI output clock timing/reference is unimportant for capture-only apps...
	if (!mConfig.fDoMultiFormat)						//	...if not sharing the device...
		mDevice.SetReference(NTV2_REFERENCE_FREERUN);	//	...let it free-run

	//	Set the device video format to whatever was detected at the input...
	mDevice.SetVideoFormat (mVideoFormat, false, false, mConfig.fInputChannel);
	mDevice.SetVANCMode (NTV2_VANCMODE_OFF, mConfig.fInputChannel);	//	Disable VANC

		//	Set the frame buffer pixel format for the FrameStore to be used on the device...
		mDevice.SetFrameBufferFormat (mConfig.fInputChannel, mConfig.fPixelFormat);
		if (mDevice.features().CanDoRP188())
		{
			bool rp188BypassEnabled(false);
			const bool readBypassOK(mDevice.IsRP188BypassEnabled(mConfig.fInputChannel, rp188BypassEnabled));
			bool disableBypassOK(true);
			if (readBypassOK && rp188BypassEnabled)
				disableBypassOK = mDevice.DisableRP188Bypass(mConfig.fInputChannel);
			const bool modeOK(mDevice.SetRP188Mode(mConfig.fInputChannel, NTV2_RP188_INPUT));
			const bool filterOK(mDevice.SetRP188SourceFilter(mConfig.fInputChannel, 0));
			cerr << "AJA_LIVE_CAPTURE_RP188_SETUP"
				 << " channel=" << DEC(UWord(mConfig.fInputChannel) + 1)
				 << " supported=1"
				 << " bypassWas=" << (rp188BypassEnabled ? 1 : 0)
				 << " disableBypass=" << (disableBypassOK ? 1 : 0)
				 << " modeInput=" << (modeOK ? 1 : 0)
				 << " sourceFilter=" << (filterOK ? 1 : 0)
				 << endl;
		}
		else
		{
			cerr << "AJA_LIVE_CAPTURE_RP188_SETUP channel=" << DEC(UWord(mConfig.fInputChannel) + 1)
				 << " supported=0" << endl;
		}
		if (!mDevice.SetTsiFrameEnable(mUseLevelBTSIRouting, mConfig.fInputChannel))
		{
		cerr << "## WARNING: failed to set TSI frame mode to "
			 << (mUseLevelBTSIRouting ? 1 : 0)
			 << " on channel " << DEC(UWord(mConfig.fInputChannel) + 1)
			 << endl;
	}
	cerr << "AJA_LIVE_CAPTURE_ROUTING mode="
		 << (mUseLevelBTSIRouting ? "tsi-ds1ds2" : "single-link")
		 << " forceTsi=" << (forceLevelBTsiRouting ? 1 : 0)
		 << " channel=" << DEC(UWord(mConfig.fInputChannel) + 1)
		 << endl;
	return AJA_STATUS_SUCCESS;

}	//	SetupVideo


AJAStatus NTV2Capture::SetupAudio (void)
{
	//	In multiformat mode, base the audio system on the channel...
	if (mConfig.fDoMultiFormat)
		if (mDevice.features().GetNumAudioSystems() > 1)
			if (UWord(mConfig.fInputChannel) < mDevice.features().GetNumAudioSystems())
				mAudioSystem = ::NTV2ChannelToAudioSystem(mConfig.fInputChannel);

	NTV2AudioSystemSet audSystems (::NTV2MakeAudioSystemSet (mAudioSystem, 1));
	NTV2AudioSource audioSource (NTV2_AUDIO_EMBEDDED);
	if (NTV2_INPUT_SOURCE_IS_HDMI(mConfig.fInputSource))
		audioSource = NTV2_AUDIO_HDMI;

	UWord failures (0);
	const UWord numAudChannels (mDevice.features().GetMaxAudioChannels());
	for (NTV2AudioSystemSetConstIter it(audSystems.begin()); it != audSystems.end(); ++it)
	{
		const NTV2AudioSystem audSys (*it);
		if (!mDevice.SetAudioSystemInputSource(audSys, audioSource, ::NTV2InputSourceToEmbeddedAudioInput(mConfig.fInputSource)))
			failures++;
		if (!mDevice.SetNumberAudioChannels(numAudChannels, audSys)) failures++;
		if (!mDevice.SetAudioRate(NTV2_AUDIO_48K, audSys)) failures++;
		if (!mDevice.SetAudioBufferSize(NTV2_AUDIO_BUFFER_SIZE_4MB, audSys)) failures++;
		if (!mDevice.SetAudioLoopBack(NTV2_AUDIO_LOOPBACK_OFF, audSys)) failures++;
		cerr << "AJA_LIVE_CAPTURE_AUDIO_ROUTE"
			 << " input=" << ::NTV2InputSourceToString(mConfig.fInputSource, true)
			 << " source=" << ::NTV2AudioSourceToString(audioSource, true)
			 << " audioSystem=" << ::NTV2AudioSystemToString(audSys, true)
			 << " channels=" << DEC(numAudChannels)
			 << endl;
	}
	if (failures)
		return AJA_STATUS_FAIL;
	return AJA_STATUS_SUCCESS;

}	//	SetupAudio


void NTV2Capture::SetupHostBuffers (void)
{
	//	Let my circular buffer know when it's time to quit...
	mAVCircularBuffer.SetAbortFlag(&mGlobalQuit);

		ULWord F1AncSize(0), F2AncSize(0);
		if (mConfig.fWithAnc)
		{	//	Use the max anc size stipulated by the AncFieldOffset VReg values...
				if (NTV2_INPUT_SOURCE_IS_HDMI(mConfig.fInputSource) && mDevice.features().CanDoHDMIAuxCapture())
				{
					const bool isProgressive(NTV2_VIDEO_FORMAT_HAS_PROGRESSIVE_PICTURE(mVideoFormat));
					const ULWord hdmiAuxMaxSize(256 * 1024);
					if (isProgressive)
					mDevice.AncSetFrameBufferSize(hdmiAuxMaxSize, 0);
				else
					mDevice.AncSetFrameBufferSize(hdmiAuxMaxSize / 2, hdmiAuxMaxSize / 2);
			}
			ULWord	F1OffsetFromEnd(0), F2OffsetFromEnd(0);
			mDevice.ReadRegister(kVRegAncField1Offset, F1OffsetFromEnd);	//	# bytes from end of 8MB/16MB frame
			mDevice.ReadRegister(kVRegAncField2Offset, F2OffsetFromEnd);	//	# bytes from end of 8MB/16MB frame
		//	Based on the offsets, calculate the max anc capacity
		F1AncSize = F2OffsetFromEnd > F1OffsetFromEnd ? 0 : F1OffsetFromEnd - F2OffsetFromEnd;
		F2AncSize = F2OffsetFromEnd > F1OffsetFromEnd ? F2OffsetFromEnd - F1OffsetFromEnd : F2OffsetFromEnd;
	}

	//	Allocate and add each in-host NTV2FrameData to my circular buffer member variable...
	const size_t audioBufferSize (NTV2_AUDIOSIZE_MAX);
	mHostBuffers.reserve(size_t(CIRCULAR_BUFFER_SIZE));
	while (mHostBuffers.size() < size_t(CIRCULAR_BUFFER_SIZE))
	{
		mHostBuffers.push_back(NTV2FrameData());
		NTV2FrameData & frameData(mHostBuffers.back());
		frameData.fVideoBuffer.Allocate(mFormatDesc.GetVideoWriteSize());
		frameData.fAudioBuffer.Allocate(NTV2_IS_VALID_AUDIO_SYSTEM(mAudioSystem) ? audioBufferSize : 0);
		frameData.fAncBuffer.Allocate(F1AncSize);
		frameData.fAncBuffer2.Allocate(F2AncSize);
		mAVCircularBuffer.Add(&frameData);

#ifdef NTV2_BUFFER_LOCK
		// Page lock the memory
		if (frameData.fVideoBuffer)
			mDevice.DMABufferLock(frameData.fVideoBuffer, true);
		if (frameData.fAudioBuffer)
			mDevice.DMABufferLock(frameData.fAudioBuffer, true);
		if (frameData.fAncBuffer)
			mDevice.DMABufferLock(frameData.fAncBuffer, true);
#endif
	}	//	for each NTV2FrameData

}	//	SetupHostBuffers


bool NTV2Capture::RouteInputSignal (void)
{
	NTV2LHIHDMIColorSpace inputColorSpace (NTV2_LHIHDMIColorSpaceYCbCr);
	if (NTV2_INPUT_SOURCE_IS_HDMI(mConfig.fInputSource))
		mDevice.GetHDMIInputColor (inputColorSpace, mConfig.fInputChannel);

	const bool isInputRGB (inputColorSpace == NTV2_LHIHDMIColorSpaceRGB);
	const bool isFrameRGB (::IsRGBFormat(mConfig.fPixelFormat));

	// For 1080p50/59.94/60 Level-B SDI, route DS1+DS2 through a TSI mux into
	// FrameStore A/B inputs so both links are consumed from a single SDI spigot.
	if (mUseLevelBTSIRouting && NTV2_INPUT_SOURCE_IS_SDI(mConfig.fInputSource))
	{
		NTV2XptConnections conns;
		const NTV2ChannelList tsiMuxes(CNTV2DemoCommon::GetTSIMuxesForFrameStore(mDevice, mConfig.fInputChannel, 1));
		if (!tsiMuxes.empty())
		{
			const NTV2Channel tsiMux(tsiMuxes.at(0));
			const NTV2OutputXptID sdiDS1(::GetInputSourceOutputXpt(mConfig.fInputSource, /*isSDI_DS2*/false, isInputRGB));
			const NTV2OutputXptID sdiDS2(::GetInputSourceOutputXpt(mConfig.fInputSource, /*isSDI_DS2*/true, isInputRGB));
			const NTV2InputXptID fbA(::GetFrameStoreInputXptFromChannel(mConfig.fInputChannel, /*isBInput*/false));
			const NTV2InputXptID fbB(::GetFrameStoreInputXptFromChannel(mConfig.fInputChannel, /*isBInput*/true));
			const NTV2InputXptID muxA(::GetTSIMuxInputXptFromChannel(tsiMux, /*linkB*/false));
			const NTV2InputXptID muxB(::GetTSIMuxInputXptFromChannel(tsiMux, /*linkB*/true));
			const NTV2OutputXptID muxOutA(::GetTSIMuxOutputXptFromChannel(tsiMux, /*linkB*/false, /*rgb*/isFrameRGB));
			const NTV2OutputXptID muxOutB(::GetTSIMuxOutputXptFromChannel(tsiMux, /*linkB*/true, /*rgb*/isFrameRGB));

			if (isInputRGB == isFrameRGB)
			{
				conns.insert(NTV2Connection(muxA, sdiDS1));
				conns.insert(NTV2Connection(muxB, sdiDS2));
			}
			else
			{
				const NTV2Channel cscA(mConfig.fInputChannel);
				const NTV2Channel cscB(NTV2Channel(UWord(mConfig.fInputChannel) + 1));
				const bool haveDualCSC(mDevice.features().GetNumCSCs() > UWord(cscB));
				if (!haveDualCSC)
				{
					cerr << "## WARNING: Level-B TSI routing requested but only one CSC available; falling back to default routing" << endl;
				}
				else
				{
					conns.insert(NTV2Connection(::GetCSCInputXptFromChannel(cscA), sdiDS1));
					conns.insert(NTV2Connection(::GetCSCInputXptFromChannel(cscB), sdiDS2));
					conns.insert(NTV2Connection(muxA, ::GetCSCOutputXptFromChannel(cscA, /*key*/false, /*rgb*/isFrameRGB)));
					conns.insert(NTV2Connection(muxB, ::GetCSCOutputXptFromChannel(cscB, /*key*/false, /*rgb*/isFrameRGB)));
				}
			}

			if (!conns.empty())
			{
				conns.insert(NTV2Connection(fbA, muxOutA));
				conns.insert(NTV2Connection(fbB, muxOutB));
				cerr << "AJA_LIVE_CAPTURE_ROUTING mode=tsi-ds1ds2"
					 << " tsiMux=" << DEC(UWord(tsiMux) + 1)
					 << " frameRGB=" << (isFrameRGB ? 1 : 0)
					 << " inputRGB=" << (isInputRGB ? 1 : 0)
					 << endl;
				return mDevice.ApplySignalRoute(conns, !mConfig.fDoMultiFormat);
			}
		}
		cerr << "## WARNING: Level-B TSI routing unavailable; using default routing" << endl;
	}

	NTV2XptConnections connections;
	return CNTV2DemoCommon::GetInputRouting(connections, mConfig, isInputRGB)
		&& mDevice.ApplySignalRoute(connections, !mConfig.fDoMultiFormat);

}	//	RouteInputSignal


AJAStatus NTV2Capture::Run (void)
{
	//	Start the playout and capture threads...
	StartConsumerThread();
	StartProducerThread();
	return AJA_STATUS_SUCCESS;

}	//	Run


//////////////////////////////////////////////////////////////////////////////////////////////////////////

//	This starts the consumer thread
void NTV2Capture::StartConsumerThread (void)
{
	//	Create and start the consumer thread...
	mConsumerThread.Attach(ConsumerThreadStatic, this);
	mConsumerThread.SetPriority(AJA_ThreadPriority_High);
	mConsumerThread.Start();

}	//	StartConsumerThread


//	The consumer thread function
void NTV2Capture::ConsumerThreadStatic (AJAThread * pThread, void * pContext)		//	static
{
	(void) pThread;

	//	Grab the NTV2Capture instance pointer from the pContext parameter,
	//	then call its ConsumeFrames method...
	NTV2Capture *	pApp (reinterpret_cast<NTV2Capture*>(pContext));
	pApp->ConsumeFrames();

}	//	ConsumerThreadStatic


void NTV2Capture::ConsumeFrames (void)
{
	CAPNOTE("Thread started");
	AJA_NTV2_AUDIO_RECORD_BEGIN	//	Active when AJA_RAW_AUDIO_RECORD or AJA_WAV_AUDIO_RECORD defined
	uint64_t ancTally(0);
	ofstream * pOFS(mConfig.fAncDataFilePath.empty() ? AJA_NULL : new ofstream(mConfig.fAncDataFilePath.c_str(), ios::binary));
	while (!mGlobalQuit)
	{
		//	Wait for the next frame to become ready to "consume"...
		NTV2FrameData *	pFrameData(mAVCircularBuffer.StartConsumeNextBuffer());
		if (pFrameData)
		{
			//	Do something useful with the frame data...
			//	. . .		. . .		. . .		. . .
			//		. . .		. . .		. . .		. . .
			//			. . .		. . .		. . .		. . .
			AJA_NTV2_AUDIO_RECORD_DO	//	Active when AJA_RAW_AUDIO_RECORD or AJA_WAV_AUDIO_RECORD defined
			if (pOFS  &&  !ancTally++)
				cerr << "Writing raw anc to '" << mConfig.fAncDataFilePath << "', "
					<< DEC(pFrameData->AncBufferSize() + pFrameData->AncBuffer2Size())
					<< " bytes per frame" << endl;
			if (pOFS  &&  pFrameData->AncBuffer())
				pOFS->write(pFrameData->AncBuffer(), streamsize(pFrameData->AncBufferSize()));
				if (pOFS  &&  pFrameData->AncBuffer2())
					pOFS->write(pFrameData->AncBuffer2(), streamsize(pFrameData->AncBuffer2Size()));
					EmitTimecodeTelemetry(*pFrameData);
					if (mConfig.fWithAnc)
						EmitCameraMetadataTelemetry(*pFrameData);
					if (mConfig.fWithAudio)
						EmitAudioTelemetry(*pFrameData);
					if (mRawStreamEnabled || mFrameMapEnabled)
						WriteStreamFrame(*pFrameData);
				if (!mSnapshotPath.empty()  &&  !mSnapshotCaptured)
				{
					mSnapshotCaptured = WriteSnapshotFrame(*pFrameData);
					mGlobalQuit = true;
				}

				//	Now release and recycle the buffer...
				mAVCircularBuffer.EndConsumeNextBuffer();
			}	//	if pFrameData
	}	//	loop til quit signaled
	if (pOFS)
		{delete pOFS; cerr << "Wrote " << DEC(ancTally) << " frames of raw anc data" << endl;}
	AJA_NTV2_AUDIO_RECORD_END	//	Active when AJA_RAW_AUDIO_RECORD or AJA_WAV_AUDIO_RECORD defined
	CAPNOTE("Thread completed, will exit");

}	//	ConsumeFrames


void NTV2Capture::EmitTimecodeTelemetry (NTV2FrameData & inFrameData)
{
	const uint64_t nowMs (AJATime::GetSystemMilliseconds());
	if (mLastTimecodeLogMs && nowMs - mLastTimecodeLogMs < 1000)
		return;

	const FrameTimecodeTelemetry tc (ExtractFrameTimecodeTelemetry(inFrameData, mVideoFormat));
	cerr << "AJA_LIVE_CAPTURE_TIMECODE"
		 << " valid=" << (tc.valid ? 1 : 0)
		 << " source=" << DEC(tc.source)
		 << " tc=" << tc.timecode
		 << " userBits=" << tc.userBits
		 << " dbb=0x" << hex << tc.raw.fDBB
		 << " lo=0x" << tc.raw.fLo
		 << " hi=0x" << tc.raw.fHi << dec
		 << endl;
	mLastTimecodeTelemetry = tc.timecode + "|" + tc.userBits;
	mLastTimecodeLogMs = nowMs;
}


void NTV2Capture::EmitAudioTelemetry (NTV2FrameData & inFrameData)
{
	const uint64_t nowMs (AJATime::GetSystemMilliseconds());
	const bool audioEnabled (mConfig.fWithAudio && NTV2_IS_VALID_AUDIO_SYSTEM(mAudioSystem));
	ULWord audioBytes (audioEnabled ? inFrameData.fNumAudioBytes : 0);
	ULWord numChannels (0);
	NTV2AudioRate audioRate (NTV2_AUDIO_RATE_INVALID);
	double sampleRateHz (0.0);
	NTV2AudioChannelPairs nonPcmPairs;

	if (audioEnabled)
	{
		mDevice.GetNumberAudioChannels(numChannels, mAudioSystem);
		mDevice.GetAudioRate(audioRate, mAudioSystem);
		sampleRateHz = ::GetAudioSamplesPerSecond(audioRate);
		mDevice.GetInputAudioChannelPairsWithoutPCM(mConfig.fInputChannel, nonPcmPairs);
	}

	ostringstream nonPcmStream;
	nonPcmStream << nonPcmPairs;
	const string nonPcmText (SanitizedMetadataValue(nonPcmStream.str()));
	const bool detected (audioEnabled && audioBytes > 0 && sampleRateHz > 0.0 && numChannels > 0);
		string peakDbText ("-");
		string rmsDbText ("-");
		AudioLTCProbeResult ltcProbe;
		ltcProbe.candidateChannels = "-";
		ltcProbe.zeroCrossHz = "-";
		ltcProbe.rmsDb = "-";
		ltcProbe.confidence = "-";

		if (detected && inFrameData.AudioBuffer())
		{
			const size_t channelCount (size_t(min<ULWord>(numChannels, 16)));
			const size_t bytesPerSample (sizeof(int32_t));
		const size_t bytesPerFrame (bytesPerSample * size_t(numChannels));
		const size_t sampleFrames (bytesPerFrame > 0 ? size_t(audioBytes) / bytesPerFrame : 0);
		if (channelCount > 0 && sampleFrames > 0)
			{
				vector<double> peak (channelCount, 0.0);
				vector<double> sumSquares (channelCount, 0.0);
				vector<double> rmsDbByChannel (channelCount, -90.0);
				const int32_t * samples (reinterpret_cast<const int32_t*>(inFrameData.AudioBuffer().GetHostPointer()));
				const double fullScale (2147483648.0);
				for (size_t frameIndex = 0; frameIndex < sampleFrames; frameIndex++)
				{
				const size_t frameOffset (frameIndex * size_t(numChannels));
				for (size_t channelIndex = 0; channelIndex < channelCount; channelIndex++)
				{
					const double sampleValue = double(samples[frameOffset + channelIndex]);
					const double normalized (min(1.0, fabs(sampleValue) / fullScale));
					peak[channelIndex] = max(peak[channelIndex], normalized);
					sumSquares[channelIndex] += normalized * normalized;
				}
			}
			const auto toDb = [](const double value) -> double
			{
				return max(-90.0, min(0.0, 20.0 * log10(max(value, 1.0e-9))));
			};
			ostringstream peakStream;
			ostringstream rmsStream;
			peakStream << fixed << setprecision(1);
			rmsStream << fixed << setprecision(1);
				for (size_t channelIndex = 0; channelIndex < channelCount; channelIndex++)
				{
					if (channelIndex > 0)
					{
						peakStream << ",";
					rmsStream << ",";
					}
					const double rms (sqrt(sumSquares[channelIndex] / double(sampleFrames)));
					rmsDbByChannel[channelIndex] = toDb(rms);
					peakStream << toDb(peak[channelIndex]);
					rmsStream << rmsDbByChannel[channelIndex];
				}
				peakDbText = peakStream.str();
				rmsDbText = rmsStream.str();
				ltcProbe = ProbeAudioLTCCandidates(samples, sampleFrames, size_t(numChannels), channelCount, sampleRateHz, rmsDbByChannel);
			}
		}

	ostringstream signature;
	signature << (detected ? 1 : 0)
			  << "|" << DEC(audioBytes)
			  << "|" << DEC(numChannels)
			  << "|" << fixed << setprecision(0) << sampleRateHz
				  << "|" << nonPcmText
				  << "|" << peakDbText
				  << "|" << rmsDbText
				  << "|" << ltcProbe.candidateChannels
				  << "|" << ltcProbe.zeroCrossHz
				  << "|" << ltcProbe.confidence;
	const string signatureText (signature.str());

		uint64_t audioMeterIntervalMs (160);
		if (const char * envAudioMeterMs = getenv("REPLAYASSIST_AJA_AUDIO_METER_MS"))
		{
			const unsigned long parsedInterval = strtoul(envAudioMeterMs, NULL, 10);
			if (parsedInterval >= 50 && parsedInterval <= 1000)
				audioMeterIntervalMs = parsedInterval;
		}
		if (mLastAudioLogMs && nowMs - mLastAudioLogMs < audioMeterIntervalMs)
			return;

	cerr << "AJA_LIVE_CAPTURE_AUDIO_SIGNAL"
		 << " detected=" << (detected ? 1 : 0)
		 << " sampleRate=" << fixed << setprecision(0) << sampleRateHz
		 << " channels=" << DEC(numChannels)
		 << " bitDepth=32"
		 << " format=s32le"
		 << " bytes=" << DEC(audioBytes)
			 << " nonPcmPairs=" << nonPcmText
			 << " peakDb=" << peakDbText
			 << " rmsDb=" << rmsDbText
			 << " ltcCandidateChannels=" << SanitizedMetadataValue(ltcProbe.candidateChannels)
			 << " ltcZeroCrossHz=" << SanitizedMetadataValue(ltcProbe.zeroCrossHz)
			 << " ltcRmsDb=" << SanitizedMetadataValue(ltcProbe.rmsDb)
			 << " ltcConfidence=" << SanitizedMetadataValue(ltcProbe.confidence)
			 << endl;

	mLastAudioTelemetry = signatureText;
	mLastAudioLogMs = nowMs;
}


void NTV2Capture::EmitCameraMetadataTelemetry (NTV2FrameData & inFrameData)
{
		const uint64_t nowMs (AJATime::GetSystemMilliseconds());
		const bool useHDMIAux (NTV2_INPUT_SOURCE_IS_HDMI(mConfig.fInputSource) && mDevice.features().CanDoHDMIAuxCapture());
		const FrameCameraMetadataTelemetry metadata (ExtractFrameCameraMetadataTelemetry(inFrameData, useHDMIAux));
	const string signature(metadata.cameraVendor + "|" + metadata.cameraModel + "|" + metadata.recordFlag + "|" + metadata.ancPacketIDs);
	if (mLastCameraMetadataLogMs && nowMs - mLastCameraMetadataLogMs < 1000)
		return;

			cerr << "AJA_LIVE_CAPTURE_CAMERA_METADATA"
			 << " cameraVendor=" << SanitizedMetadataValue(metadata.cameraVendor)
			 << " cameraModel=" << SanitizedMetadataValue(metadata.cameraModel)
			 << " cameraID=" << SanitizedMetadataValue(metadata.cameraID)
			 << " cameraSerial=" << SanitizedMetadataValue(metadata.cameraSerial)
			 << " cameraFirmware=" << SanitizedMetadataValue(metadata.cameraFirmware)
			 << " lensData=" << SanitizedMetadataValue(metadata.lensData)
			 << " isoEI=" << SanitizedMetadataValue(metadata.isoEI)
			 << " shutter=" << SanitizedMetadataValue(metadata.shutter)
		 << " whiteBalance=" << SanitizedMetadataValue(metadata.whiteBalance)
		 << " tint=" << SanitizedMetadataValue(metadata.tint)
		 << " nd=" << SanitizedMetadataValue(metadata.nd)
			 << " aperture=" << SanitizedMetadataValue(metadata.aperture)
			 << " focusDistance=" << SanitizedMetadataValue(metadata.focusDistance)
			 << " cameraMediaFilename=" << SanitizedMetadataValue(metadata.cameraMediaFilename)
			 << " cameraClipName=" << SanitizedMetadataValue(metadata.cameraClipName)
			 << " reelName=" << SanitizedMetadataValue(metadata.reelName)
			 << " codec=" << SanitizedMetadataValue(metadata.codec)
			 << " recordingFormat=" << SanitizedMetadataValue(metadata.recordingFormat)
			 << " sensorFPS=" << SanitizedMetadataValue(metadata.sensorFPS)
			 << " projectFPS=" << SanitizedMetadataValue(metadata.projectFPS)
			 << " colorSpace=" << SanitizedMetadataValue(metadata.colorSpace)
			 << " gamma=" << SanitizedMetadataValue(metadata.gamma)
			 << " gamut=" << SanitizedMetadataValue(metadata.gamut)
			 << " lutName=" << SanitizedMetadataValue(metadata.lutName)
			 << " lookName=" << SanitizedMetadataValue(metadata.lookName)
			 << " anamorphicSqueeze=" << SanitizedMetadataValue(metadata.anamorphicSqueeze)
			 << " recordFlag=" << SanitizedMetadataValue(metadata.recordFlag)
			 << " cameraTimecode=" << SanitizedMetadataValue(metadata.cameraTimecode)
			 << " cameraMetadataConfidence=" << SanitizedMetadataValue(metadata.cameraMetadataConfidence)
		 << " ancBytes=" << DEC(metadata.ancBytes)
		 << " ancPackets=" << DEC(metadata.ancPackets)
		 << " ancHash=" << SanitizedMetadataValue(metadata.ancHash)
		 << " ancPacketIDs=" << SanitizedMetadataValue(metadata.ancPacketIDs)
			 << " ancPayloadTypes=" << SanitizedMetadataValue(metadata.ancPayloadTypes)
				 << " ancTextPreview=" << SanitizedMetadataValue(metadata.ancTextPreview)
			 << " ancRawPrefixHex=" << SanitizedMetadataValue(metadata.ancRawPrefixHex)
				 << " rdd18PacketIDs=" << SanitizedMetadataValue(metadata.rdd18PacketIDs)
			 << " rdd18PayloadHex=" << SanitizedMetadataValue(metadata.rdd18PayloadHex)
			 << " sdiKLVPacketIDs=" << SanitizedMetadataValue(metadata.sdiKLVPacketIDs)
			 << " sdiKLVPayloadHex=" << SanitizedMetadataValue(metadata.sdiKLVPayloadHex)
			 << " sdiKLVSummary=" << SanitizedMetadataValue(metadata.sdiKLVSummary)
			 << " arriMetadataPacketIDs=" << SanitizedMetadataValue(metadata.arriMetadataPacketIDs)
			 << " arriMetadataPayloadHex=" << SanitizedMetadataValue(metadata.arriMetadataPayloadHex)
			 << " arriMetadataSummary=" << SanitizedMetadataValue(metadata.arriMetadataSummary)
			 << " sonyMetadataPacketIDs=" << SanitizedMetadataValue(metadata.sonyMetadataPacketIDs)
			 << " sonyMetadataPayloadHex=" << SanitizedMetadataValue(metadata.sonyMetadataPayloadHex)
			 << " sonyMetadataSummary=" << SanitizedMetadataValue(metadata.sonyMetadataSummary)
			 << " canonMetadataPacketIDs=" << SanitizedMetadataValue(metadata.canonMetadataPacketIDs)
			 << " canonMetadataPayloadHex=" << SanitizedMetadataValue(metadata.canonMetadataPayloadHex)
			 << " canonMetadataSummary=" << SanitizedMetadataValue(metadata.canonMetadataSummary)
			 << " redMetadataPacketIDs=" << SanitizedMetadataValue(metadata.redMetadataPacketIDs)
			 << " redMetadataPayloadHex=" << SanitizedMetadataValue(metadata.redMetadataPayloadHex)
			 << " redMetadataSummary=" << SanitizedMetadataValue(metadata.redMetadataSummary)
		 << " blackmagicTallyPacketIDs=" << SanitizedMetadataValue(metadata.blackmagicTallyPacketIDs)
		 << " blackmagicTallyPayloadHex=" << SanitizedMetadataValue(metadata.blackmagicTallyPayloadHex)
		 << " blackmagicTallySummary=" << SanitizedMetadataValue(metadata.blackmagicTallySummary)
		 << " blackmagicTallyMonitor=" << SanitizedMetadataValue(metadata.blackmagicTallyMonitor)
		 << " blackmagicTallyVersion=" << SanitizedMetadataValue(metadata.blackmagicTallyVersion)
		 << " blackmagicTallySlaveStates=" << SanitizedMetadataValue(metadata.blackmagicTallySlaveStates)
		 << " blackmagicCCUPacketIDs=" << SanitizedMetadataValue(metadata.blackmagicCCUPacketIDs)
		 << " blackmagicCCUPayloadHex=" << SanitizedMetadataValue(metadata.blackmagicCCUPayloadHex)
		 << " blackmagicCCUSummary=" << SanitizedMetadataValue(metadata.blackmagicCCUSummary)
		 << " blackmagicCCUDecodedFields=" << SanitizedMetadataValue(metadata.blackmagicCCUDecodedFields)
		 << " blackmagicVideoMode=" << SanitizedMetadataValue(metadata.blackmagicVideoMode)
		 << " blackmagicRecordingFormat=" << SanitizedMetadataValue(metadata.blackmagicRecordingFormat)
		 << " blackmagicCodec=" << SanitizedMetadataValue(metadata.blackmagicCodec)
		 << " blackmagicTransportMode=" << SanitizedMetadataValue(metadata.blackmagicTransportMode)
		 << " blackmagicDynamicRange=" << SanitizedMetadataValue(metadata.blackmagicDynamicRange)
		 << " blackmagicDisplayLUT=" << SanitizedMetadataValue(metadata.blackmagicDisplayLUT)
		 << " blackmagicTimecodeSource=" << SanitizedMetadataValue(metadata.blackmagicTimecodeSource)
		 << " blackmagicReference=" << SanitizedMetadataValue(metadata.blackmagicReference)
		 << " blackmagicAudio=" << SanitizedMetadataValue(metadata.blackmagicAudio)
		 << " blackmagicOutput=" << SanitizedMetadataValue(metadata.blackmagicOutput)
		 << " blackmagicDisplay=" << SanitizedMetadataValue(metadata.blackmagicDisplay)
		 << " blackmagicColorCorrection=" << SanitizedMetadataValue(metadata.blackmagicColorCorrection)
		 << " blackmagicConfiguration=" << SanitizedMetadataValue(metadata.blackmagicConfiguration)
		 << " blackmagicPTZ=" << SanitizedMetadataValue(metadata.blackmagicPTZ)
		 << endl;
	mLastCameraMetadataTelemetry = signature;
	mLastCameraMetadataLogMs = nowMs;
}


bool NTV2Capture::WriteSnapshotFrame (NTV2FrameData & inFrameData)
{
	if (mSnapshotPath.empty())
		return false;
	if (!inFrameData.fVideoBuffer || !inFrameData.fVideoBuffer.GetHostPointer() || !inFrameData.fVideoBuffer.GetByteCount())
		return false;
	if (!NTV2_IS_FBF_RGB(mConfig.fPixelFormat))
	{
		cerr << "## ERROR: snapshot requires RGB frame buffer format" << endl;
		return false;
	}

	const ULWord width (mFormatDesc.GetRasterWidth());
	const ULWord height (mFormatDesc.GetVisibleRasterHeight());
	const ULWord bytesPerRow (mFormatDesc.GetBytesPerRow(0));
	const ULWord bytesNeeded (bytesPerRow * height);
	if (!width || !height || bytesPerRow < width * 4 || inFrameData.fVideoBuffer.GetByteCount() < bytesNeeded)
		return false;

	ofstream ofs(mSnapshotPath.c_str(), ios::binary | ios::trunc);
	if (!ofs.good())
		return false;

	const uint8_t * src (reinterpret_cast<const uint8_t *>(inFrameData.fVideoBuffer.GetHostPointer()));
	ofs.write(reinterpret_cast<const char *>(src), streamsize(bytesNeeded));
	ofs.flush();
	if (!ofs.good())
		return false;

	cout << "SNAPSHOT width=" << DEC(width)
		 << " height=" << DEC(height)
		 << " bytes_per_row=" << DEC(bytesPerRow)
		 << " path=" << mSnapshotPath
		 << endl;
	return true;
}


namespace
{
static inline void PutU32LE (uint8_t * pDst, const uint32_t value)
{
	pDst[0] = uint8_t(value & 0xFF);
	pDst[1] = uint8_t((value >> 8) & 0xFF);
	pDst[2] = uint8_t((value >> 16) & 0xFF);
	pDst[3] = uint8_t((value >> 24) & 0xFF);
}

static inline void PutU64LE (uint8_t * pDst, const uint64_t value)
{
	PutU32LE(pDst, uint32_t(value & 0xFFFFFFFFULL));
	PutU32LE(pDst + 4, uint32_t((value >> 32) & 0xFFFFFFFFULL));
}

static bool WriteAll (const int fd, const uint8_t * pData, const size_t byteCount)
{
	size_t written (0);
	while (written < byteCount)
	{
		const ssize_t count (::write(fd, pData + written, byteCount - written));
		if (count <= 0)
			return false;
		written += size_t(count);
	}
	return true;
}
}	//	anonymous namespace


bool NTV2Capture::WriteStreamFrame (NTV2FrameData & inFrameData)
{
	if (!mRawStreamEnabled && !mFrameMapEnabled)
		return false;
	++mRawStreamSeenCount;
	if (mRawStreamEveryN > 1  &&  (mRawStreamSeenCount % uint64_t(mRawStreamEveryN)) != 0)
		return true;	//	Skip this frame to reduce stdout backpressure
	if (!inFrameData.fVideoBuffer || !inFrameData.fVideoBuffer.GetHostPointer() || !inFrameData.fVideoBuffer.GetByteCount())
		return false;
	if (!NTV2_IS_FBF_RGB(mConfig.fPixelFormat))
		return false;

	const ULWord width (mFormatDesc.GetRasterWidth());
	const ULWord height (mFormatDesc.GetVisibleRasterHeight());
	const ULWord srcBytesPerRow (mFormatDesc.GetBytesPerRow(0));
	const ULWord scale = mRawStreamDownsample > 0 ? mRawStreamDownsample : 1;
	const ULWord outWidth = width / scale;
	const ULWord outHeight = height / scale;
	const ULWord tightBytesPerRow (outWidth * 4);
	const ULWord srcPayloadBytes (srcBytesPerRow * height);
	if (!width || !height || !outWidth || !outHeight || srcBytesPerRow < width * 4 || inFrameData.fVideoBuffer.GetByteCount() < srcPayloadBytes)
		return false;
	const ULWord outPayloadBytes (tightBytesPerRow * outHeight);
	const uint64_t frameIndex (++mRawStreamFrameCount);

	uint8_t header[28];
	::memcpy(header, "RAWS", 4);
	PutU32LE(header + 4, uint32_t(outWidth));
	PutU32LE(header + 8, uint32_t(outHeight));
	PutU32LE(header + 12, uint32_t(tightBytesPerRow));
	PutU32LE(header + 16, uint32_t(outPayloadBytes));
	PutU64LE(header + 20, frameIndex);

	const uint8_t * pSrc (reinterpret_cast<const uint8_t *>(inFrameData.fVideoBuffer.GetHostPointer()));
	const bool directCopy = (scale == 1 && srcBytesPerRow == tightBytesPerRow);
	const uint8_t * pPayload (pSrc);
	if (!directCopy)
	{
		mRawScratch.resize(size_t(outPayloadBytes));
		uint8_t * pDst (mRawScratch.data());
		for (ULWord outY = 0; outY < outHeight; ++outY)
		{
			ULWord inY = outY * scale;
			if (inY >= height)
				inY = height - 1;
			const uint8_t * pInRow = pSrc + size_t(inY * srcBytesPerRow);
			uint8_t * pOutRow = pDst + size_t(outY * tightBytesPerRow);
			if (scale == 1)
			{
				::memcpy(pOutRow, pInRow, size_t(tightBytesPerRow));
			}
			else
			{
				for (ULWord outX = 0; outX < outWidth; ++outX)
				{
					const ULWord inX = outX * scale;
					const uint8_t * pInPx = pInRow + size_t(inX * 4);
					uint8_t * pOutPx = pOutRow + size_t(outX * 4);
					::memcpy(pOutPx, pInPx, 4);
				}
			}
		}
		pPayload = mRawScratch.data();
	}
	if (mFrameMapEnabled && mFrameMapPtr && mFrameMapSize >= (size_t(outPayloadBytes) + 64))
	{
		uint8_t * pMap = mFrameMapPtr;
		const uint64_t nextSeq = (mFrameMapSeq + 2ULL);
		const bool doubleBuffered = mFrameMapSize >= (size_t(outPayloadBytes) * 2 + 64);
		const uint32_t payloadStride = doubleBuffered ? uint32_t(outPayloadBytes) : 0U;
		const uint32_t nextBank = doubleBuffered ? ((mFrameMapBank ^ 1U) & 1U) : 0U;
		const size_t payloadOffset = 64 + size_t(nextBank) * size_t(payloadStride);
		if (payloadOffset + size_t(outPayloadBytes) <= mFrameMapSize)
		{
			const FrameTimecodeTelemetry tc (ExtractFrameTimecodeTelemetry(inFrameData, mVideoFormat));
			// Mark frame-map state as "update in progress" before payload copy.
			PutU64LE(pMap + 24, nextSeq | 1ULL);	// odd => metadata/payload update in progress
			__sync_synchronize();
			::memcpy(pMap + payloadOffset, pPayload, size_t(outPayloadBytes));
			PutU32LE(pMap + 8, uint32_t(outWidth));
			PutU32LE(pMap + 12, uint32_t(outHeight));
			PutU32LE(pMap + 16, uint32_t(tightBytesPerRow));
			PutU32LE(pMap + 20, uint32_t(outPayloadBytes));
			PutU32LE(pMap + 28, nextBank);
			PutU32LE(pMap + 32, payloadStride);
			PutCString(pMap + 36, 12, tc.valid ? tc.timecode : string());
			PutCString(pMap + 48, 12, tc.userBits);
			pMap[60] = tc.valid ? 1 : 0;
			pMap[61] = uint8_t(max(0, min(255, tc.source)));
			pMap[62] = 0;
			pMap[63] = 0;
			__sync_synchronize();
			PutU64LE(pMap + 24, nextSeq);	// even => frame committed
			mFrameMapSeq = nextSeq;
			mFrameMapBank = nextBank;
		}
		if (!mRawStreamEnabled)
			return true;
	}

	return WriteAll(STDOUT_FILENO, header, sizeof(header))
		&& WriteAll(STDOUT_FILENO, pPayload, size_t(outPayloadBytes));
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////

//	This starts the capture (producer) thread
void NTV2Capture::StartProducerThread (void)
{
	//	Create and start the capture thread...
	mProducerThread.Attach(ProducerThreadStatic, this);
	mProducerThread.SetPriority(AJA_ThreadPriority_High);
	mProducerThread.Start();

}	//	StartProducerThread


//	The capture thread function
void NTV2Capture::ProducerThreadStatic (AJAThread * pThread, void * pContext)		//	static
{
	(void) pThread;

	//	Grab the NTV2Capture instance pointer from the pContext parameter,
	//	then call its CaptureFrames method...
	NTV2Capture * pApp (reinterpret_cast<NTV2Capture*>(pContext));
	pApp->CaptureFrames ();

}	//	ProducerThreadStatic


void NTV2Capture::CaptureFrames (void)
{
		AUTOCIRCULATE_TRANSFER	inputXfer;	//	AutoCirculate input transfer info
		NTV2AudioChannelPairs	nonPcmPairs, oldNonPcmPairs;
		ULWord					acOptions (AUTOCIRCULATE_WITH_RP188), overruns(0);
		UWord					sdiSpigot (UWord(::NTV2InputSourceToChannel(mConfig.fInputSource)));
		const bool				hdmiInput (NTV2_INPUT_SOURCE_IS_HDMI(mConfig.fInputSource));
		const bool				hdmiAuxSupported (hdmiInput && mDevice.features().CanDoHDMIAuxCapture());
		const bool				useHDMIAux (mConfig.fWithAnc && hdmiAuxSupported);
		const NTV2AudioSystem	acAudioSystem (NTV2_IS_VALID_AUDIO_SYSTEM(mAudioSystem) ? mAudioSystem : NTV2_AUDIOSYSTEM_INVALID);
		if (mConfig.fWithAnc)
			acOptions |= AUTOCIRCULATE_WITH_ANC;
		if (useHDMIAux)
			acOptions |= AUTOCIRCULATE_WITH_HDMIAUX;
		if (mConfig.fWithAnc && hdmiInput)
		{
			cerr << "AJA_LIVE_CAPTURE_HDMI_METADATA"
				 << " hdmiAuxSupported=" << (hdmiAuxSupported ? 1 : 0)
				 << " mode=" << (useHDMIAux ? "hdmi-aux" : "anc-fallback")
				 << endl;
		}

	CAPNOTE("Thread started");

	//	Initialize and start capture AutoCirculate...
	mDevice.AutoCirculateStop(mConfig.fInputChannel);	//	Just in case
	if (!mDevice.AutoCirculateInitForInput (mConfig.fInputChannel, mConfig.fFrames, acAudioSystem, acOptions))
	{
		cerr << "## ERROR: AutoCirculateInitForInput failed (channel=" << DEC(UWord(mConfig.fInputChannel) + 1)
			 << " audioSystem=" << DEC(UWord(acAudioSystem)) << " options=0x" << hex << acOptions << dec << ")" << endl;
		mGlobalQuit = true;
		}
		if (!mGlobalQuit && useHDMIAux)
		{
			mDevice.AuxExtractSetFilterInclusionMode(sdiSpigot, false);
			NTV2DIDSet zeroSet;
			zeroSet.insert(0);
			mDevice.AuxExtractSetPacketFilters(sdiSpigot, zeroSet);
		}
		if (!mGlobalQuit  &&  !mDevice.AutoCirculateStart(mConfig.fInputChannel))
		{
		cerr << "## ERROR: AutoCirculateStart failed (channel=" << DEC(UWord(mConfig.fInputChannel) + 1) << ")" << endl;
		mGlobalQuit = true;
	}

	//	Ingest frames til Quit signaled...
	while (!mGlobalQuit)
	{
		AUTOCIRCULATE_STATUS acStatus;
		mDevice.AutoCirculateGetStatus (mConfig.fInputChannel, acStatus);

		if (acStatus.IsRunning()  &&  acStatus.HasAvailableInputFrame())
		{
			//	At this point, there's at least one fully-formed frame available in the device's
			//	frame buffer to transfer to the host. Reserve an NTV2FrameData to "produce", and
			//	use it in the next transfer from the device...
			NTV2FrameData *	pFrameData (mAVCircularBuffer.StartProduceNextBuffer());
			if (!pFrameData)
				continue;

			NTV2FrameData & frameData (*pFrameData);
			inputXfer.SetVideoBuffer (frameData.VideoBuffer(), frameData.VideoBufferSize());
			if (acStatus.WithAudio())
				inputXfer.SetAudioBuffer (frameData.AudioBuffer(), frameData.AudioBufferSize());
			if (acStatus.WithCustomAnc())
				inputXfer.SetAncBuffers (frameData.AncBuffer(), frameData.AncBufferSize(),
										 frameData.AncBuffer2(), frameData.AncBuffer2Size());

			//	Transfer video/audio/anc from the device into our host buffers...
			mDevice.AutoCirculateTransfer (mConfig.fInputChannel, inputXfer);

			//	Remember the actual amount of audio captured...
			if (acStatus.WithAudio())
				frameData.fNumAudioBytes = inputXfer.GetCapturedAudioByteCount();

			//	If capturing Anc, clear stale anc data from the anc buffers...
				if (acStatus.WithCustomAnc()  &&  frameData.AncBuffer())
				{	bool overrun(false);
					if (useHDMIAux)
						mDevice.AuxExtractGetBufferOverrun (sdiSpigot, overrun);
					else
						mDevice.AncExtractGetBufferOverrun (sdiSpigot, overrun);
					if (overrun)
						{overruns++;  CAPWARN(overruns << (useHDMIAux ? " aux" : " anc") << " overrun(s)");}
				frameData.fNumAncBytes = inputXfer.GetCapturedAncByteCount(/*isF2*/false);
				NTV2Buffer stale (frameData.fAncBuffer.GetHostAddress(frameData.fNumAncBytes),
									frameData.fAncBuffer.GetByteCount() - frameData.fNumAncBytes);
				stale.Fill(uint8_t(0));
			}
			if (acStatus.WithCustomAnc()  &&  frameData.AncBuffer2())
			{
				frameData.fNumAnc2Bytes = inputXfer.GetCapturedAncByteCount(/*isF2*/true);
				NTV2Buffer stale (frameData.fAncBuffer2.GetHostAddress(frameData.fNumAnc2Bytes),
									frameData.fAncBuffer2.GetByteCount() - frameData.fNumAnc2Bytes);
				stale.Fill(uint8_t(0));
			}

			//	Grab all valid timecodes that were captured...
			inputXfer.GetInputTimeCodes (frameData.fTimecodes, mConfig.fInputChannel, /*ValidOnly*/ true);

			if (acStatus.WithAudio())
				//	Look for PCM/NonPCM changes in the audio stream...
				if (mDevice.GetInputAudioChannelPairsWithoutPCM (mConfig.fInputChannel, nonPcmPairs))
				{
					NTV2AudioChannelPairs becomingNonPCM, becomingPCM;
					set_difference (oldNonPcmPairs.begin(), oldNonPcmPairs.end(),  nonPcmPairs.begin(), nonPcmPairs.end(),  inserter(becomingPCM, becomingPCM.begin()));
					set_difference (nonPcmPairs.begin(), nonPcmPairs.end(),  oldNonPcmPairs.begin(), oldNonPcmPairs.end(),  inserter(becomingNonPCM, becomingNonPCM.begin()));
					if (!becomingNonPCM.empty())
						CAPNOTE("Audio channel pair(s) '" << becomingNonPCM << "' now non-PCM");
					if (!becomingPCM.empty())
						CAPNOTE("Audio channel pair(s) '" << becomingPCM << "' now PCM");
					oldNonPcmPairs = nonPcmPairs;
				}

			//	Signal that we're done "producing" the frame, making it available for future "consumption"...
			mAVCircularBuffer.EndProduceNextBuffer();
		}	//	if A/C running and frame(s) are available for transfer
		else
		{
			//	Either AutoCirculate is not running, or there were no frames available on the device to transfer.
			//	Rather than waste CPU cycles spinning, waiting until a frame becomes available, it's far more
			//	efficient to wait for the next input vertical interrupt event to get signaled...
			mDevice.WaitForInputVerticalInterrupt(mConfig.fInputChannel);
		}

		//	Log SDI input CRC/VPID/TRS errors...
		if (mDevice.features().CanDoSDIErrorChecks() && NTV2_INPUT_SOURCE_IS_SDI(mConfig.fInputSource))
		{
			NTV2SDIInStatistics	sdiStats;
			NTV2SDIInputStatus	inputStatus;
			if (mDevice.ReadSDIStatistics(sdiStats))
			{
				sdiStats.GetSDIInputStatus(inputStatus, UWord(::GetIndexForNTV2InputSource(mConfig.fInputSource)));
				if (!inputStatus.mLocked)
					CAPWARN(inputStatus);
			}
		}
	}	//	loop til quit signaled

	//	Stop AutoCirculate...
	mDevice.AutoCirculateStop(mConfig.fInputChannel);
	CAPNOTE("Thread completed, will exit");

}	//	CaptureFrames


void NTV2Capture::GetACStatus (ULWord & outGoodFrames, ULWord & outDroppedFrames, ULWord & outBufferLevel)
{
	AUTOCIRCULATE_STATUS status;
	mDevice.AutoCirculateGetStatus(mConfig.fInputChannel, status);
	outGoodFrames    = status.GetProcessedFrameCount();
	outDroppedFrames = status.GetDroppedFrameCount();
	outBufferLevel   = status.GetBufferLevel();
}
