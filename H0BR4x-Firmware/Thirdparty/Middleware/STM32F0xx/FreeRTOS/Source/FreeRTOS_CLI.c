/*
 BitzOS (BOS) V0.2.7 - Copyright (C) 2017-2022 Hexabitz
 All rights reserved
 
 File Name     : FreeRTOS_CLI.c
 Description   : an add-on component to FreeRTOS.
 
 */
/* Standard includes. */
#include <string.h>
#include <stdint.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Utils includes. */
#include "FreeRTOS_CLI.h"

#include "BOS.h"

typedef struct xCOMMAND_INPUT_LIST
{
	const CLI_Command_Definition_t *pxCommandLineDefinition;
	struct xCOMMAND_INPUT_LIST *pxNext;
} CLI_Definition_List_Item_t;

/*
 * The callback function that is executed when "help" is entered.  This is the
 * only default command that is always present.
 */
static portBASE_TYPE prvHelpCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString );

/*
 * Return the number of parameters that follow the command name.
 */
static int8_t prvGetNumberOfParameters( const int8_t * pcCommandString );

/* The definition of the "help" command.  This command is always at the front
of the list of registered commands. */
static const CLI_Command_Definition_t xHelpCommand =		
{
	( const int8_t * ) "help",
	( const int8_t * ) "\r\nhelp:\r\n Lists all the registered commands. Type 'help params' to list available parameters and their possible values\r\n\r\n",
	prvHelpCommand,
	-1 /* Variable number of parameters is expected. */
};

/* The definition of the list of commands.  Commands that are registered are
added to this list. */
CLI_Definition_List_Item_t xRegisteredCommands =		/* Hexabitz >> removed static */
{
	&xHelpCommand,	/* The first command in the list is always the help command, defined in this file. */
	NULL			/* The next pointer is initialised to NULL, as there are no other registered commands yet. */
};

/* A buffer into which command outputs can be written is declared here, rather
than in the command console implementation, to allow multiple command consoles
to share the same buffer.  For example, an application may allow access to the
command interpreter by UART and by Ethernet.  Sharing a buffer is done purely
to save RAM.  Note, however, that the command console itself is not re-entrant,
so only one command interpreter interface can be used at any one time.  For that
reason, no attempt at providing mutual exclusion to the cOutputBuffer array is
attempted. */
static int8_t cOutputBuffer[ configCOMMAND_INT_MAX_OUTPUT_SIZE ];

/*-----------------------------------------------------------*/

portBASE_TYPE FreeRTOS_CLIRegisterCommand( const CLI_Command_Definition_t * const pxCommandToRegister )
{
static CLI_Definition_List_Item_t *pxLastCommandInList = &xRegisteredCommands;
CLI_Definition_List_Item_t *pxNewListItem;
portBASE_TYPE xReturn = pdFAIL;

	/* Check the parameter is not NULL. */
	configASSERT( pxCommandToRegister );

	/* Create a new list item that will reference the command being registered. */
	pxNewListItem = ( CLI_Definition_List_Item_t * ) pvPortMalloc( sizeof( CLI_Definition_List_Item_t ) );
	configASSERT( pxNewListItem );

	if( pxNewListItem != NULL )
	{
		taskENTER_CRITICAL();
		{
			/* Reference the command being registered from the newly created
			list item. */
			pxNewListItem->pxCommandLineDefinition = pxCommandToRegister;

			/* The new list item will get added to the end of the list, so
			pxNext has nowhere to point. */
			pxNewListItem->pxNext = NULL;

			/* Add the newly created list item to the end of the already existing
			list. */
			pxLastCommandInList->pxNext = pxNewListItem;

			/* Set the end of list marker to the new list item. */
			pxLastCommandInList = pxNewListItem;
		}
		taskEXIT_CRITICAL();

		xReturn = pdPASS;
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

portBASE_TYPE FreeRTOS_CLIProcessCommand( const int8_t * const pcCommandInput, int8_t * pcWriteBuffer, size_t xWriteBufferLen  )
{
static const CLI_Definition_List_Item_t *pxCommand = NULL;
portBASE_TYPE xReturn = pdTRUE;
const int8_t *pcRegisteredCommandString;
size_t xCommandStringLength;

	/* Note:  This function is not re-entrant.  It must not be called from more
	than one task. */

	if( pxCommand == NULL )
	{
		/* Search for the command string in the list of registered commands. */
		for( pxCommand = &xRegisteredCommands; pxCommand != NULL; pxCommand = pxCommand->pxNext )
		{
			pcRegisteredCommandString = pxCommand->pxCommandLineDefinition->pcCommand;
			xCommandStringLength = strlen( ( const char * ) pcRegisteredCommandString );

			/* To ensure the string lengths match exactly, so as not to pick up
			a sub-string of a longer command, check the byte after the expected
			end of the string is either the end of the string or a space before
			a parameter. */
			if( ( pcCommandInput[ xCommandStringLength ] == ' ' ) || ( pcCommandInput[ xCommandStringLength ] == 0x00 ) )
			{
				if( strncmp( ( const char * ) pcCommandInput, ( const char * ) pcRegisteredCommandString, xCommandStringLength ) == 0 )
				{
					/* The command has been found.  Check it has the expected
					number of parameters.  If cExpectedNumberOfParameters is -1,
					then there could be a variable number of parameters and no
					check is made. */
					if( pxCommand->pxCommandLineDefinition->cExpectedNumberOfParameters >= 0 )
					{
						if( prvGetNumberOfParameters( pcCommandInput ) != pxCommand->pxCommandLineDefinition->cExpectedNumberOfParameters )
						{
							xReturn = pdFALSE;
						}
					}

					break;
				}
			}
		}
	}

	if( ( pxCommand != NULL ) && ( xReturn == pdFALSE ) )
	{
		/* The command was found, but the number of parameters with the command
		was incorrect. */
		strncpy( ( char * ) pcWriteBuffer, "Incorrect command parameter(s).  Enter \"help\" to view a list of available commands.\r\n\r\n", xWriteBufferLen );
		pxCommand = NULL;
	}
	else if( pxCommand != NULL )
	{
		/* Call the callback function that is registered to this command. */
		xReturn = pxCommand->pxCommandLineDefinition->pxCommandInterpreter( pcWriteBuffer, xWriteBufferLen, pcCommandInput );

		/* If xReturn is pdFALSE, then no further strings will be returned
		after this one, and	pxCommand can be reset to NULL ready to search
		for the next entered command. */
		if( xReturn == pdFALSE )
		{
			pxCommand = NULL;
		}
	}
	else
	{
		/* pxCommand was NULL, the command was not found. */
		strncpy( ( char * ) pcWriteBuffer, ( const char * ) "Command not recognised.  Enter \"help\" to view a list of available commands.\r\n\r\n", xWriteBufferLen );
		xReturn = pdFALSE;
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

int8_t *FreeRTOS_CLIGetOutputBuffer( void )
{
	return cOutputBuffer;
}
/*-----------------------------------------------------------*/

const int8_t *FreeRTOS_CLIGetParameter( const int8_t *pcCommandString, unsigned portBASE_TYPE uxWantedParameter, portBASE_TYPE *pxParameterStringLength )
{
unsigned portBASE_TYPE uxParametersFound = 0;
const int8_t *pcReturn = NULL;

	*pxParameterStringLength = 0;

	while( uxParametersFound < uxWantedParameter )
	{
		/* Index the character pointer past the current word.  If this is the start
		of the command string then the first word is the command itself. */
		while( ( ( *pcCommandString ) != 0x00 ) && ( ( *pcCommandString ) != ' ' ) )
		{
			pcCommandString++;
		}

		/* Find the start of the next string. */
		while( ( ( *pcCommandString ) != 0x00 ) && ( ( *pcCommandString ) == ' ' ) )
		{
			pcCommandString++;
		}

		/* Was a string found? */
		if( *pcCommandString != 0x00 )
		{
			/* Is this the start of the required parameter? */
			uxParametersFound++;

			if( uxParametersFound == uxWantedParameter )
			{
				/* How long is the parameter? */
				pcReturn = pcCommandString;
				while( ( ( *pcCommandString ) != 0x00 ) && ( ( *pcCommandString ) != ' ' ) )
				{
					( *pxParameterStringLength )++;
					pcCommandString++;
				}

				if( *pxParameterStringLength == 0 )
				{
					pcReturn = NULL;
				}

				break;
			}
		}
		else
		{
			break;
		}
	}

	return pcReturn;
}
/*-----------------------------------------------------------*/

static portBASE_TYPE prvHelpCommand( int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString )
{
static const CLI_Definition_List_Item_t * pxCommand = NULL;
signed portBASE_TYPE xReturn; 
static uint8_t str, counter;
static int8_t *pcParameterString1; 
portBASE_TYPE xParameterStringLength1 = 0;
	
	/* Obtain the 1st parameter string: The set parameter */
	pcParameterString1 = ( int8_t * ) FreeRTOS_CLIGetParameter (pcCommandString, 1, &xParameterStringLength1);

	
	/* List parameters */
	if (pcParameterString1 != NULL && !strncmp((const char *)pcParameterString1, "params", 6))
	{
		if( str == NumOfParamsHelpStrings )
		{
			/* Reset the pointer back to the start of the list. */
			str = 0;
		}	
	
		/* Return the next command help string, before moving the pointer on to
		the next command in the list. */
		strncpy( ( char * ) pcWriteBuffer, ( const char * )pcParamsHelpString[str++], xWriteBufferLen );

		if( str == NumOfParamsHelpStrings )
		{
			/* There are no more parameters in the list, so there will be no more
			strings to return after this one and pdFALSE should be returned. */
			xReturn = pdFALSE; str = 0;
		}
		else
		{
			xReturn = pdTRUE;
		}		
	}
	/* List CLI Commands */
	else if (pcParameterString1 == NULL)
	{		
		if( pxCommand == NULL )
		{
			/* Reset the pxCommand pointer back to the start of the list. */
			pxCommand = &xRegisteredCommands;
			counter = 1;
		}

		/* Return the next command help string, before moving the pointer on to
		the next command in the list. */	
		if (counter > numOfBosCommands) {
			strncpy( 8 + ( char * ) pcWriteBuffer, ( const char * ) pxCommand->pxCommandLineDefinition->pcHelpString, xWriteBufferLen-8 );
			strcpy(1 + ( char * ) pcWriteBuffer, ( char * ) modulePNstring[modulePN]);
			pcWriteBuffer[0] = '('; pcWriteBuffer[6] = ')'; pcWriteBuffer[7] = ' ';
		} else {
			strncpy( ( char * ) pcWriteBuffer, ( const char * ) pxCommand->pxCommandLineDefinition->pcHelpString, xWriteBufferLen );
		}				
				
		pxCommand = pxCommand->pxNext;
		counter++;

		if( pxCommand == NULL )
		{
			/* There are no more commands in the list, so there will be no more
			strings to return after this one and pdFALSE should be returned. */
			xReturn = pdFALSE;
		}
		else
		{
			xReturn = pdTRUE;
		}
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

static int8_t prvGetNumberOfParameters( const int8_t * pcCommandString )
{
int8_t cParameters = 0;
portBASE_TYPE xLastCharacterWasSpace = pdFALSE;

	/* Count the number of space delimited words in pcCommandString. */
	while( *pcCommandString != 0x00 )
	{
		if( ( *pcCommandString ) == ' ' )
		{
			if( xLastCharacterWasSpace != pdTRUE )
			{
				cParameters++;
				xLastCharacterWasSpace = pdTRUE;
			}
		}
		else
		{
			xLastCharacterWasSpace = pdFALSE;
		}

		pcCommandString++;
	}

	/* If the command string ended with spaces, then there will have been too
	many parameters counted. */
	if( xLastCharacterWasSpace == pdTRUE )
	{
		cParameters--;
	}

	/* The value returned is one less than the number of space delimited words,
	as the first word should be the command itself. */
	return cParameters;
}

