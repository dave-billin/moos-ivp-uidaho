#==============================================================================
# FILE: CMakeUtils.cmake
# DESCRIPTION: Some useful utility functions and macros for Cmake debugging
#
# Created 8-2011 by Dave Billin
#==============================================================================

#==========================
# Print()
#==========================
# Prints the name of a variable and its string contents
#
# EXAMPLE:
#   The CMake code:
#       set(MYVAR "FiddleFaddle")
#       Print(MYVAR)
#
#   results in the following output:
#       MYVAR = FiddleFaddle
macro( Print TARGETVAR )
    message("${TARGETVAR} = ${${TARGETVAR}}")
endmacro()



#==========================
# PrintList()
#==========================
# Prints the name of a specified list followed by that list's contents
#
# EXAMPLE:   PrintList(MYLIST)
#   results in the following output:
#
# 	MYLIST = 
#      	<list item 1>
#       <list item 2>
#           ...
#       <list item n>
#
function( PrintList TARGETLIST )
          message("${TARGETLIST} = ")
          foreach ( ITEM ${${TARGETLIST}} )
              message( "\t${ITEM}" )
          endforeach ( ITEM ${TARGETLIST} ) 
endfunction(PrintList)




#==========================
# ListToString()
#==========================
# This macro converts a CMake semicolon-delimited list to a string
# containing elements separated by spaces.
# USAGE:	ListToString(VARIABLE, <dest var>) 
# 	
# Where VARIABLE is the name of the CMAKE list variable to convert
# and <dest var> is the variable to be populated with the resulting
# string.
#
macro (ListToString TARGETLIST DESTVAR)
    foreach(ITEM ${${TARGETLIST}})
        set(${DESTVAR} "${${DESTVAR}} ${ITEM}")
    endforeach()   
endmacro()
