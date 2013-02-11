/******************************************************************************

  USB Host Human Interface Device Parser Header File

Summary:
    This is the header file for a USB Embedded Host that is Human Interface
    Device Class . This header file contains HID parser related informaton.

Description:
    This is the header file for a USB Embedded Host that is Human Interface
    Device Class .

    This file should be included with usb_host.h to provide the USB hardware
    interface. It must be included after the application-specific usb_config.h
    file and after the USB Embedded Host header file usb_host.h, as definitions
    in those files are required for proper compilation. This file contains HID
	parser related definitions.

	Acronyms/abbreviations used by this class:
    * HID - Human Interface Device

	Every HID class device identifies itself with a report descriptor. A Report
	descriptor describes each piece of data that the device generates and
	what the data is actually measuring. A parser is needed to decode the
	content of report descriptor.
	
	Report descriptors are composed of pieces of information. Each piece of
	information is called an Item.
	HID Item Header
	
	---------------------------------------------------------
	|  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |
	|            Tag            |    Type     |    Size     |
	---------------------------------------------------------

	The HID class driver contains a parser used to analyze items found in the Report
	descriptor. The parser extracts information from the descriptor in a linear fashion.
	The parser collects the state of each known item as it walks through the
	descriptor, and stores them in an item state table.

	Any HID device to be compliant must have a valid report descriptor.
	Micochip HID host stack comes with a HID parser that does basic sanity check
	and provides interface functions to understand the reports transmitted
	from the device. Refer HID firmware specifications to understand parser.
	This file contains data structures that stores parsed information in more
	accessible format.
	

*******************************************************************************/
//DOM-IGNORE-BEGIN
/*******************************************************************************

* FileName:        usb_host_hid_parser.h
* Dependencies:    None
* Processor:       PIC24/dsPIC30/dsPIC33/PIC32MX
* Compiler:        C30 v2.01/C32 v0.00.18
* Company:         Microchip Technology, Inc.

Software License Agreement

The software supplied herewith by Microchip Technology Incorporated
(the “Company”) for its PICmicro® Microcontroller is intended and
supplied to you, the Company’s customer, for use solely and
exclusively on Microchip PICmicro Microcontroller products. The
software is owned by the Company and/or its supplier, and is
protected under applicable copyright laws. All rights are reserved.
Any use in violation of the foregoing restrictions may subject the
user to criminal sanctions under applicable laws, as well as to
civil liability for the breach of the terms and conditions of this
license.

THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

Author          Date    Comments
--------------------------------------------------------------------------------
ADG          9-Apr-2008 First release
*******************************************************************************/
//DOM-IGNORE-END

//DOM-IGNORE-BEGIN
#ifndef _USB_HOST_HID_PARSER_H_ /* usb_host_hid_parser.h */
#define _USB_HOST_HID_PARSER_H_ 
//DOM-IGNORE-END


#define HIDItem_SizeMask           0x03     // Mask for Size bitfield in Item header
#define HIDItem_TagMask            0xF0     // Mask for Tag bitfield in Item header
#define HIDItem_TagShift           0x04     // Shift Value for Tag bitfield in Item header
#define HIDItem_TypeMask           0xC      // Mask for Type bitfield in Item header
#define HIDItem_TypeShift          0x02     // Shift Value for Tag bitfield

//------------------------------------------------------------------------------
//
// HID Item Type Definitions
//
//------------------------------------------------------------------------------

#define HIDType_Main               0        // Main Item value
#define HIDType_Global             1        // Global Item value
#define HIDType_Local              2        // Local Item value
#define HIDType_Long               3        // Long Item value

//------------------------------------------------------------------------------
//
// HID Item Tag Definitions - Main Items
//
//------------------------------------------------------------------------------

#define HIDTag_Input               0x08     // Input Tag value
#define HIDTag_Output              0x09     // Output Tag value
#define HIDTag_Collection          0x0A     // Collection Tag value
#define HIDTag_Feature             0x0B     // Feature Tag value
#define HIDTag_EndCollection       0x0C     // End of Collection Tag value

//------------------------------------------------------------------------------
//
// HID Item Tag Definitions - Globals
//
//------------------------------------------------------------------------------
#define HIDTag_UsagePage           0x00     // UsagePage Tag value
#define HIDTag_LogicalMinimum      0x01     // Logical Minimum Tag value 
#define HIDTag_LogicalMaximum      0x02     // Logical Maximum Tag value
#define HIDTag_PhysicalMinimum     0x03     // Physical Minimum Tag value
#define HIDTag_PhysicalMaximum     0x04     // Physical Maximum Tag value
#define HIDTag_UnitExponent        0x05     // Unit Exponent Tag value
#define HIDTag_Unit                0x06     // Unit  Tag value               
#define HIDTag_ReportSize          0x07     // Report Size  Tag value     
#define HIDTag_ReportID            0x08     // Report ID  Tag value            
#define HIDTag_ReportCount         0x09     // ReportCount  Tag value
#define HIDTag_Push                0x0A     // Push  Tag value
#define HIDTag_Pop                 0x0B     // Pop  Tag value     

//------------------------------------------------------------------------------
//                                     
// HID Item Tag Definitions - Locals
//
//------------------------------------------------------------------------------

#define HIDTag_Usage               0x00     //  Usage Tag value  
#define HIDTag_UsageMinimum        0x01     //  Usage Minimum Tag value     
#define HIDTag_UsageMaximum        0x02     //  Usage Maximum Tag value
#define HIDTag_DesignatorIndex     0x03     //  Designator Index Tag value
#define HIDTag_DesignatorMinimum   0x04     //  Designator Minimum Tag value
#define HIDTag_DesignatorMaximum   0x05     //  Designator Maximum Tag value
#define HIDTag_StringIndex         0x07     //  String Index Tag value
#define HIDTag_StringMinimum       0x08     //  String Minimum Tag value
#define HIDTag_StringMaximum       0x09     //  String Maximum Tag value
#define HIDTag_SetDelimiter        0x0A     //  Set Delimiter Tag value

//------------------------------------------------------------------------------
//
// HID Main Item Header Bit Definitions
//
//------------------------------------------------------------------------------

#define HIDData_BufferedBytes     0x100     //  HID data bytes are bufferred
#define HIDData_VolatileBit        0x80     //  Volatile bit position
#define HIDData_Volatile           0x80     //  HID data is voaltile
#define HIDData_NullStateBit       0x40     //  NULL state bit position
#define HIDData_NullState          0x40     //  NULL state defined
#define HIDData_NoPreferredBit     0x20     //  No Preferred bt position
#define HIDData_NoPreferred        0x20     //  HID data type No Preferred
#define HIDData_NonlinearBit       0x10     //  NonLinear bit position
#define HIDData_Nonlinear          0x10     //  HID data type NonLinear
#define HIDData_WrapBit            0x08     //  Wrap bit position
#define HIDData_Wrap               0x08     //  HID data type Wrap
#define HIDData_RelativeBit        0x04     //  Relative bit position
#define HIDData_Relative           0x04     //  HID data type relative
#define HIDData_Absolute           0x00     //  HID data type absolute
#define HIDData_VariableBit        0x02     //  Variable bit position
#define HIDData_Variable           0x02     //  HID data type variable 
#define HIDData_ArrayBit           0x02     //  Array Bit position 
#define HIDData_Array              0x00     //  Array indentifier value
#define HIDData_ConstantBit        0x01     //  Constant Bit position       
#define HIDData_Constant           0x01     //  Constant data type indentifier value

//------------------------------------------------------------------------------
//
// HID Collection Data Definitions
//
//------------------------------------------------------------------------------
#define HIDCollection_Physical     0x00
#define HIDCollection_Application  0x01


typedef enum {
    hidReportInput,
    hidReportOutput,
    hidReportFeature,
    hidReportUnknown
} HIDReportTypeEnum;


    
// *****************************************************************************
/* HID Item Information

This structure contains information about each Item of the report descriptor.
*/
typedef struct _HID_ITEM_INFO
{
   union
    {
       struct 
       {
           BYTE             ItemSize :2;        // Numeric expression specifying size of data 
           BYTE             ItemType :2;        // This field identifies type of item(Main, Global or Local)
           BYTE             ItemTag  :4;        // This field specifies the function of the item
       };
       BYTE                 val;                // to access the data in byte format
    }                       ItemDetails;    

    union
    {
        LONG                    sItemData;      // Item Data is stored in signed format
        DWORD                   uItemData;      // Item Data is stored in unsigned format
        BYTE                    bItemData[4];
    }   Data;
}   HID_ITEM_INFO; 


// *****************************************************************************
/* HID Global Item Information

This structure contains information about each Global Item of the report descriptor.
*/
typedef struct _HID_GLOBALS
{
    WORD                     usagePage;         // Specifies current Usage Page
    LONG                     logicalMinimum;    // This is the minimum value that a variable or array item will report
    LONG                     logicalMaximum;    // This is the maximum value that a variable or array item will report
    LONG                     physicalMinimum;   // Minimum value for the physical extent of a variable item
    LONG                     physicalMaximum;   // Maximum value for the physical extent of a variable item
    LONG                     unitExponent;      // Value of the unit exponent in base 10
    LONG                     unit;              // Unit values
    WORD                     reportIndex;       // Conter to keep track of report being processed in the parser
    BYTE                     reportID;          // Report ID. All the reports are preceded by a single byte report ID
    BYTE                     reportsize;        // Size of current report in bytes
    BYTE                     reportCount;       // This field determines number of fields in the report

}   HID_GLOBALS;

// *****************************************************************************
/* HID Report details

This structure contains information about each report exchanged with the device.
*/
typedef struct _HID_REPORT
{
    WORD                     reportID;          // Report ID of the associated report
    WORD                     inputBits;         // If input report then length of report in bits
    WORD                     outputBits;        // If output report then length of report in bits
    WORD                     featureBits;       // If feature report then length of report in bits
}   HID_REPORT;


// *****************************************************************************
/* HID Collection Details

This structure contains information about each collection encountered in the report descriptor.
*/
typedef struct _HID_COLLECTION
{
    DWORD                    data;              // Collection raw data 
    WORD                     usagePage;         // Usage page associated with current level of collection
    BYTE                     firstUsageItem;    // Index of First Usage Item in the current collection
    BYTE                     usageItems;        // Number of Usage Items in the current collection
    BYTE                     firstReportItem;   // Index of First report Item in the current collection
    BYTE                     reportItems;       // Number of report Items in the current collection
    BYTE                     parent;            // Index to Parent collection
    BYTE                     firstChild;        // Index to next child collection in the report descriptor
    BYTE                     nextSibling;       // Index to next child collection in the report descriptor
}   HID_COLLECTION;

// *****************************************************************************
/* HID Report Details

This structure contains information about each Report encountered in the report descriptor.
*/
typedef struct _HID_REPORTITEM
{
    HIDReportTypeEnum        reportType;          // Type of Report Input/Output/Feature
    HID_GLOBALS              globals;             // Stores all the global items associated with the current report
    BYTE                     startBit;            // Starting Bit Position of the report 
    BYTE                     parent;              // Index of parent collection
    DWORD                    dataModes;           // this tells the data mode is array or not
    BYTE                     firstUsageItem;      // Index to first usage item related to the report
    BYTE                     usageItems;          // Number of usage items in the current report
    BYTE                     firstStringItem;     // Index to first srting item in the list
    BYTE                     stringItems;         // Number of string items in the current report
    BYTE                     firstDesignatorItem; // Index to first designator item
    BYTE                     designatorItems;     // Number of designator items in the current report
}   HID_REPORTITEM;

// *****************************************************************************
/* HID Report Details

This structure contains information about each Usage Item encountered in the report descriptor.
*/
typedef struct _HID_USAGEITEM
{
    BOOL                     isRange;       // True if Usage item has a valid MAX and MIN range
    WORD                     usagePage;     // Usage page ID asscociated with the Item
    WORD                     usage;         // Usage ID asscociated with the Item
    WORD                     usageMinimum;  // Defines the starting usage associated with an array or bitmap
    WORD                     usageMaximum;  // Defines the ending usage associated with an array or bitmap
}   HID_USAGEITEM;

// *****************************************************************************
/* HID String Item Details

This structure contains information about each Report encountered in the report descriptor.
*/
typedef struct _HID_STRINGITEM
{
    BOOL                     isRange;   // If range of String Item is valid
    WORD                     index;     // String index for a String descriptor; allows a string to be associated with a particular item or control
    WORD                     minimum;   // Specifies the first string index when assigning a group of sequential strings to controls in an array or bitmap
    WORD                     maximum;   // Specifies the last string index when assigning a group of sequential strings to controls in an array or bitmap
}   HID_STRINGITEM, HID_DESIGITEM;


// *****************************************************************************
/* Report Descriptor Information

   This structure contains top level information of the report descriptor. This information
   is important and is used to understand the information during th ecourse of parsing.
   This structure also stores temporary data needed during parsing the report descriptor.
   All of this information may not be of much inportance to the application.
*/
typedef struct _USB_HID_DEVICE_RPT_INFO
{
    WORD reportPollingRate;     // This stores the pollrate for the input report. Application can use this to decide the rate of transfer
    BYTE interfaceNumber;       // This stores the interface number for the current report descriptor

    // This set of members are used during parsing of Report descriptor , application does not normally need these details
    BOOL haveDesignatorMax;     // True if report descriptor has a valid Designator Max
    BOOL haveDesignatorMin;     // True if report descriptor has a valid Designator Min
    BOOL haveStringMax;         // True if report descriptor has a valid String Max
    BOOL haveStringMin;         // True if report descriptor has a valid String Min
    BOOL haveUsageMax;          // True if report descriptor has a valid Usage Max
    BOOL haveUsageMin;          // True if report descriptor has a valid Usage Min
    WORD designatorMaximum;     // Last designator max value
    WORD designatorMinimum;     // Last designator min value
    WORD designatorRanges;      // Last designator range
    WORD designators;           // This tells toatal number of designator items
    WORD rangeUsagePage;        // current usage page during parsing
    WORD stringMaximum;         // current string maximum
    WORD stringMinimum;         // current string minimum
    WORD stringRanges;          // current string ranges
    WORD usageMaximum;          // current usage maximum
    WORD usageMinimum;          // current usage minimum
    WORD usageRanges;           // current usage ranges 
    BYTE collectionNesting;     // this number tells depth of collection nesting
    BYTE collections;           // total number of collections
    BYTE designatorItems;       // total number of designator items
    BYTE firstUsageItem;        // index of first usage item for the current collection
    BYTE firstDesignatorItem;   // index of first designator item for the current collection
    BYTE firstStringItem;       // index of first string item for the current collection
    BYTE globalsNesting;        // On encountering every PUSH item , this is incremented , keep track of current depth of Globals
    BYTE maxCollectionNesting;  // Maximum depth of collections
    BYTE maxGlobalsNesting;     // Maximum depth of Globals
    BYTE parent;                // Parent collection
    BYTE reportItems;           // total number of report items
    BYTE reports;               // total number of reports
    BYTE sibling;               // current sibling collection
    BYTE stringItems;           // total number of string items , used to index the array of strings
    BYTE strings;               // total sumber of strings
    BYTE usageItems;            // total number of usage items , used to index the array of usage
    BYTE usages;                // total sumber of usages
    HID_GLOBALS globals;        // holds cuurent globals items

}   USB_HID_DEVICE_RPT_INFO;


// *****************************************************************************
/* List of Items

   This structure contains array of pointers to all the Items in the report descriptor.
   HID parser will populate the lists while parsing the report descriptor. This data is
   used by interface functions provided in file usb_host_hid_interface.c to retrive data
   from the report received from the device. Application can also access these details
   to retreive the intended information incase provided interface function fail to do so.
*/
typedef struct _USB_HID_ITEM_LIST
{
    HID_COLLECTION *collectionList;     // List of collections, see HID_COLLECTION for details in the structure
    HID_DESIGITEM *designatorItemList;  // List of designator Items, see HID_DESIGITEM for details in the structure
    HID_GLOBALS *globalsStack;          // List of global Items, see HID_GLOBALS for details in the structure
    HID_REPORTITEM *reportItemList;     // List of report Items, see HID_REPORTITEM for details in the structure
    HID_REPORT *reportList;             // List of reports , see HID_REPORT for details in the structure
    HID_STRINGITEM *stringItemList;     // List of string item , see HID_STRINGITEM for details in the structure
    HID_USAGEITEM *usageItemList;       // List of Usage item , see HID_USAGEITEM for details in the structure
    BYTE *collectionStack;              // stores the array of parents ids for the collection
}   USB_HID_ITEM_LIST;

// *****************************************************************************
/* HID parser error codes

   This enumerates the error encountered during the parsing of report descriptor. In case of any error
   parsing is sttopped and the error is flagged. Device is not attched successfully.
*/
typedef enum {
    HID_ERR = 0,                        // No error
    HID_ERR_NotEnoughMemory,            // If not enough Heap can be allocated, make sure sufficient dynamic memory is aloocated for the parser
    HID_ERR_NullPointer,                // Pointer to report descriptor is NULL
    HID_ERR_UnexpectedEndCollection,    // End of collection not expected
    HID_ERR_UnexpectedPop,              // POP not expected
    HID_ERR_MissingEndCollection,       // No end of collection found
    HID_ERR_MissingTopLevelCollection,  // Atleast one collection must be present
    HID_ERR_NoReports,                  // atlest one report must be present
    HID_ERR_UnmatchedUsageRange,        // Either Minimum or Maximum for usage range missing
    HID_ERR_UnmatchedStringRange,       // Either Minimum or Maximum for string range missing
    HID_ERR_UnmatchedDesignatorRange,   // Either Minimum or Maximum for designator range missing
    HID_ERR_UnexpectedEndOfDescriptor,  // Report descriptor not formatted properly
    HID_ERR_BadLogicalMin,              // Logical Min greater than report size
    HID_ERR_BadLogicalMax,              // Logical Max greater than report size
    HID_ERR_BadLogical,                 // If logical Min is greater than Max
    HID_ERR_ZeroReportSize,             // Report size is zero
    HID_ERR_ZeroReportID,               // report ID is zero
    HID_ERR_ZeroReportCount,            // Number of reports is zero
    HID_ERR_BadUsageRangePage,          // Bad Usage page range
    HID_ERR_BadUsageRange               // Bad Usage range
} USB_HID_RPT_DESC_ERROR;

/****************************************************************************
  Function:
    BOOL USBHostHID_HasUsage(HID_REPORTITEM *reportItem, WORD usagePage,
                                          WORD usage, WORD *pindex, BYTE* count)

  Description:
    This function is used to locate the usage in a report descriptor.
    Function will look into the data structures created by the HID parser
    and return the appropriate location.

  Precondition:
    None

  Parameters:
    HID_REPORTITEM *reportItem - Report item index to be searched
    WORD usagePage             - Application needs to pass the usagePage as
                                 the search criteria for the usage
    WORD usage                 - Application needs to pass the usageto be
                                 searched
    WORD *pindex               - returns index to the usage item requested.
    BYTE* count                - returns the remaining number of reports

  Return Values:
    BOOL                       - FALSE - If requested usage is not found
                                 TRUE  - if requested usage is found
  Remarks:
    None
***************************************************************************/
BOOL USBHostHID_HasUsage(HID_REPORTITEM *reportItem,WORD usagePage, WORD usage,WORD *pindex,BYTE* count);


//******************************************************************************
//******************************************************************************
// Section: External Variables
//******************************************************************************
//******************************************************************************

extern USB_HID_DEVICE_RPT_INFO deviceRptInfo;
extern USB_HID_ITEM_LIST       itemListPtrs;

#endif /* usb_host_hid_parser.h */
