//=============================================================================
/*    Copyright (C) 2012  Dave Billin

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
//-----------------------------------------------------------------------------
/** @file BHV_ReverseDive.h
 *
 * @brief
 * 	A behavior for pHelmIvP used to carry out a reverse dive
 *
 * @author Dave Billin
 */
//=============================================================================
 
#ifndef BHV_REVERSEDIVE_H
#define BHV_REVERSEDIVE_H

#include <string>
#include <set>
#include <list>

#include "VarDataPair.h"
#include "IvPBehavior.h"
#include "ZAIC_HEQ.h"




//=============================================================================
/** An IvP behavior class used to implement a reverse-dive sequence in the
 * YellowSub
 */
class BHV_ReverseDive : public IvPBehavior
{
public:

	//==============================
	// Default behavior settings
	//==============================
	static const double DefaultPitchDownPitch_deg;
	static const double DefaultPitchDownTimeout_sec;
	static const double DefaultPitchDownSpeed_mps;

	static const double DefaultDiveToDepthTarget_m;
	static const double DefaultDiveToDepthTimeout_sec;
	static const double DefaultDiveToDepthSpeed_mps;

	static const double DefaultLevelOutPitch_deg;
	static const double DefaultLevelOutTimeout_sec;
	static const double DefaultLevelOutSpeed_mps;


	//=========================================================================
	/** Called to create an instance of the behavior
	 * @param Domain
	 *	An IvP Domain to operate in
	 */
	BHV_ReverseDive(IvPDomain Domain);


	//=========================================================================
	/** Called when the object goes out of scope */
	virtual ~BHV_ReverseDive();


	//=========================================================================
	/** Called to set the value of a behavior parameter
	 *
	 * @param sParameter
	 * 	A string containing the name of the parameter whose value is being set
	 *
	 * @param sParamValue
	 * 	A string containing the value of the parameter
	 *
	 * @return
	 * 	true if the parameter was set; else false on error
	 */
	bool setParam( std::string sParamName, std::string sParamValue );


	//=========================================================================
	/** Called when the behavior is running and idle (but not active) */
	void onIdleState();

	//=========================================================================
	/** Called to produce an IvP objective function when the behavior is
	 * running */
	IvPFunction* onRunState();

	//=========================================================================
	/** Called when the behavior transitions from idle to running */
	void onIdleToRunState( void );

	//=========================================================================
	/** Called when the behavior transitions from running to idle */
	void onRunToIdleState( void );

	//=========================================================================
	/** Called when the behavior registers itself as complete */
	void onCompleteState( void );




protected:

	/** @enum e_DiveStates
	 * @brief
	 * 	Dive states used in the Reverse dive
	 */
	enum e_DiveStates
	{
		INIT,			/**< Initialization state (runs for one iteration) */
		PITCH_DOWN,		/**< Drive down and aft-ward to attain a target pitch */
		DIVETODEPTH,	/**< Drive aft-ward at target pitch to attain target
							 depth */
		LEVEL_OUT,		/**< Drive aft-ward but level out to zero pitch */
		SUCCESS,		/**< Signal dive success and behavior completion */
		ERROR,			/**< Publishes error flags, re-enables controllers, and
						 	 signals behavior completion */
		NUM_DIVE_STATES	/**< Internal use only */
	};


	bool m_IsSimulation;	/**< Set to true if running under simulation; false
	 	 	 	 	 	 	 	 if running on actual vehicle */

	double m_DepthZaicPeak;	/**< peak value of the depth zaic function */

	ZAIC_HEQ* m_pDepthZaic;/**< Function used to produce an objective function
								for desired depth during the PitchDown and
								DiveToDepth states */

	double m_SpeedZaicPeak;	/**< peak value of the speed zaic function */

	ZAIC_HEQ* m_pSpeedZaic;/**< Function used to produce an objective function
								over the speed domain to drive the prop in
								reverse */

	IvPFunction* m_pObjectiveFunction;	/**< Objective function produced by the
											 behavior */

	/** Inner class used to wrap each dive state's configuration */
	struct DiveStateConfig
	{
		DiveStateConfig()
		  : Timeout_sec(0.0), Pitch_rad(0.0), Depth_m(0.0), Speed_mps(0.0)
		{}

		~DiveStateConfig()
		{}

		double Timeout_sec;	/**< State timeout in seconds */
		double Pitch_rad;	/**< Target pitch in radians */
		double Depth_m;		/**< Target depth in meters */
		double Speed_mps;	/**< Target speed in meters per second */
		list <VarDataPair> Flags;	/**< State flags */
	};

	DiveStateConfig m_StateConfig[NUM_DIVE_STATES];	/**< State datum */

	int m_DiveState;		/**< Current state in the reverse-dive sequence */
	double m_StateEntryTime;	/**< MOOSTime when the current state was
									 entered */

	int m_Verbosity;	/**< Verbosity of debugging messages */




private:

	static const std::string sm_sBhvName;	/**< Behavior name as a string */

	//=========================================================================
	/** Utility function to split a variable-value pair into separate strings */
	bool ExtractVarValue( std::string sSource, VarDataPair& VarValue );

	//=========================================================================
	/** A function to register a dive error an post all dive error flags
	 * @param String to publish containing the error description
	 */
	void HandleDiveError( std::string sError );

	//=========================================================================
	/** Utility function to publish all items in a list of Variable-value
	 * pairs.  List items are published as "repeatable" to bypass the
	 * duplication filter.
	 */
	void PublishFlagList( std::list<VarDataPair>& VdpList );

	//=========================================================================
	/** Helper function to generate new objective functions for the behavior */
	IvPFunction* GenerateObjectiveFunction( double DesiredDepth,
											double DesiredSpeed );

	// Prevent automatic generation of copy constructor and assignment operator
	BHV_ReverseDive(const BHV_ReverseDive&);
	const BHV_ReverseDive& operator= (const BHV_ReverseDive&);

};



#ifdef WIN32
	// Windows needs to explicitly specify functions to export from a dll
   #define IVP_EXPORT_FUNCTION __declspec(dllexport)
#else
   #define IVP_EXPORT_FUNCTION
#endif

extern "C"
{
  IVP_EXPORT_FUNCTION IvPBehavior* createBehavior( std::string name,
		  	  	  	  	  	  	  	  	  	  	   IvPDomain domain)
  {
	  return new BHV_ReverseDive(domain);
  }
}


#endif		// END #ifndef BHV_REVERSEDIVE_H

