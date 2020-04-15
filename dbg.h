
#ifndef INCLUDED_DBG_H
#define INCLUDED_DBG_H

#define ENTRY int ret = 0; \
              pr_info( "Entering %s\n",__FUNCTION__ );
#define ENTRYV pr_info( "Entering %s\n",__FUNCTION__ );
#define EXIT  pr_info( "Exiting %s ret = %d\n",__FUNCTION__,ret ); \
              return ret;
#define EXITV pr_info( "Exiting %s\n",__FUNCTION__ ); \
              return;

#endif
