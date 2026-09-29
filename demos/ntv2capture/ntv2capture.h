/* SPDX-License-Identifier: MIT */
/**
	@file		ntv2capture.h
	@brief		Declares the NTV2Capture class.
	@copyright	(C) 2012-2022 AJA Video Systems, Inc.  All rights reserved.
**/


#ifndef _NTV2CAPTURE_H
#define _NTV2CAPTURE_H

#include "ntv2democommon.h"
#include "ajabase/system/thread.h"
#include <string>
#include <vector>


/**
	@brief	Instances of me capture frames in real time from a video signal provided to an input of an AJA device.
**/
class NTV2Capture
{
	//	Public Instance Methods
	public:
		/**
			@brief		Constructs me using the given settings.
			@param[in]	inConfig	Specifies how to configure capture.
			@note		I'm not completely initialized and ready to use until after my Init method has been called.
		**/
		NTV2Capture (const CaptureConfig & inConfig);

		virtual						~NTV2Capture ();

		/**
			@brief	Initializes me and prepares me to Run.
		**/
		virtual AJAStatus			Init (void);

		/**
			@brief	Runs me.
			@note	Call this method only after calling Init and it returned AJA_STATUS_SUCCESS.
		**/
		virtual AJAStatus			Run (void);

		/**
			@brief	Gracefully stops me from running.
		**/
			virtual void				Quit (void);

			/**
				@brief	Enables one-shot frame snapshot mode.
				@param[in]	inFilePath	Absolute or relative path to raw BGRA output file.
			**/
			virtual void				EnableSnapshot (const std::string & inFilePath);
			virtual void				EnableRawStream (void);
			virtual void				EnableFrameMap (const std::string & inFilePath, const ULWord inMaxBytes = 0);
			virtual void				SetRawStreamFrameDecimation (const ULWord inEveryN);
			virtual void				SetRawStreamDownsample (const ULWord inFactor);

			/**
				@brief	Returns true after a snapshot frame has been written.
			**/
			virtual bool				SnapshotCaptured (void) const;
			virtual bool				RawStreamEnabled (void) const;
			virtual bool				FrameMapEnabled (void) const;

		/**
			@brief	Provides status information about my input (capture) process.
			@param[out]	outGoodFrames		Receives the number of successfully captured frames.
			@param[out]	outDroppedFrames	Receives the number of dropped frames.
			@param[out]	outBufferLevel		Receives the buffer level (number of captured frames ready to be transferred to the host).
		**/
		virtual void				GetACStatus (ULWord & outGoodFrames, ULWord & outDroppedFrames, ULWord & outBufferLevel);


	//	Protected Instance Methods
	protected:
		/**
			@brief	Sets up everything I need for capturing video.
		**/
		virtual AJAStatus		SetupVideo (void);

		/**
			@brief	Sets up everything I need for capturing audio.
		**/
		virtual AJAStatus		SetupAudio (void);

		/**
			@brief	Sets up device routing for capture.
		**/
		virtual bool			RouteInputSignal (void);

		/**
			@brief	Sets up my circular buffers.
		**/
		virtual void			SetupHostBuffers (void);

		/**
			@brief	Starts my capture thread.
		**/
		virtual void			StartProducerThread (void);

		/**
			@brief	Repeatedly captures frames using AutoCirculate (until global quit flag set).
		**/
		virtual void			CaptureFrames (void);

		/**
			@brief	Starts my frame consumer thread.
		**/
		virtual void			StartConsumerThread (void);

		/**
			@brief	Repeatedly consumes frames from the circular buffer (until global quit flag set).
		**/
			virtual void			ConsumeFrames (void);
			virtual bool			WriteSnapshotFrame (NTV2FrameData & inFrameData);
			virtual bool			WriteStreamFrame (NTV2FrameData & inFrameData);
			virtual void			EmitTimecodeTelemetry (NTV2FrameData & inFrameData);
			virtual void			EmitAudioTelemetry (NTV2FrameData & inFrameData);
			virtual void			EmitCameraMetadataTelemetry (NTV2FrameData & inFrameData);
			virtual bool			SetupFrameMap (void);
			virtual void			TeardownFrameMap (void);


	//	Protected Class Methods
	protected:
		/**
			@brief	This is the consumer thread's static callback function that gets called when the consumer thread runs.
					This function gets "Attached" to the consumer thread's AJAThread instance.
			@param[in]	pThread		A valid pointer to the consumer thread's AJAThread instance.
			@param[in]	pContext	Context information to pass to the thread.
									(For this application, this will point to the NTV2Capture instance.)
		**/
		static void	ConsumerThreadStatic (AJAThread * pThread, void * pContext);

		/**
			@brief	This is the capture thread's static callback function that gets called when the capture thread runs.
					This function gets "Attached" to the AJAThread instance.
			@param[in]	pThread		Points to the AJAThread instance.
			@param[in]	pContext	Context information to pass to the thread.
									(For this application, this will point to the NTV2Capture instance.)
		**/
		static void	ProducerThreadStatic (AJAThread * pThread, void * pContext);


	//	Private Member Data
	private:
		AJAThread			mConsumerThread;	///< @brief	My consumer thread object -- consumes the captured frames.
		AJAThread			mProducerThread;	///< @brief	My producer thread object -- does the frame capturing
		CNTV2Card			mDevice;			///< @brief	My CNTV2Card instance. This is what I use to talk to the device.
		NTV2DeviceID		mDeviceID;			///< @brief	My device identifier
		CaptureConfig		mConfig;			///< @brief	My operating configuration
		NTV2VideoFormat		mVideoFormat;		///< @brief	My video format
		NTV2FormatDesc		mFormatDesc;		///< @brief	Describes my video raster
		NTV2TaskMode		mSavedTaskMode;		///< @brief	Used to restore prior every-frame task mode
		NTV2AudioSystem		mAudioSystem;		///< @brief	The audio system I'm using (if any)
		NTV2FrameDataArray	mHostBuffers;		///< @brief	My host buffers
			FrameDataRingBuffer	mAVCircularBuffer;	///< @brief	My ring buffer object
			bool				mGlobalQuit;		///< @brief	Set "true" to gracefully stop
			bool				mUseLevelBTSIRouting;	///< @brief	True when Level-B SDI should route DS1/DS2 via TSI mux
			bool				mSnapshotCaptured;	///< @brief	Set true after writing snapshot
			std::string			mSnapshotPath;		///< @brief	Output path for one-shot snapshot
			bool				mRawStreamEnabled;	///< @brief	Set true to stream BGRA frames to stdout
			uint64_t			mRawStreamFrameCount;///< @brief	Incremented for each streamed frame
			ULWord				mRawStreamEveryN;	///< @brief	Only every Nth frame is streamed (1=no decimation)
			uint64_t			mRawStreamSeenCount;///< @brief	Incremented for each candidate stream frame
			ULWord				mRawStreamDownsample;///< @brief	2=half width/height, 1=no scaling
			std::vector<uint8_t> mRawScratch;	///< @brief	Reusable scratch buffer for scaled/deinterlaced output
			bool				mFrameMapEnabled;	///< @brief	Set true to stream latest frame into mmap file
			std::string			mFrameMapPath;		///< @brief	Memory-mapped frame file path
			ULWord				mFrameMapMaxBytes;	///< @brief	Optional max bytes requested by caller
			int					mFrameMapFD;		///< @brief	File descriptor for mapped frame file
			uint8_t *			mFrameMapPtr;		///< @brief	Mapped file base pointer
				size_t				mFrameMapSize;		///< @brief	Size of mapped frame file
				uint64_t			mFrameMapSeq;		///< @brief	Committed frame sequence number (even values)
				uint32_t			mFrameMapBank;		///< @brief	Current committed payload bank (0/1 for double-buffer maps)
				uint64_t			mLastTimecodeLogMs;	///< @brief	Last emitted RP188 telemetry timestamp
				std::string			mLastTimecodeTelemetry;	///< @brief	Last emitted RP188 telemetry payload
				uint64_t			mLastAudioLogMs;	///< @brief	Last emitted audio telemetry timestamp
				std::string			mLastAudioTelemetry;	///< @brief	Last emitted audio telemetry payload
				uint64_t			mLastCameraMetadataLogMs;	///< @brief	Last emitted camera metadata telemetry timestamp
				std::string			mLastCameraMetadataTelemetry;	///< @brief	Last emitted camera metadata telemetry payload

			};	//	NTV2Capture

#endif	//	_NTV2CAPTURE_H
