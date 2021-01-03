"""
Flag classes

Author: eYRC_SB_363
"""


class Flag(object):
    """
    - Flag class, but of course! 
    - It can be of three types 
        - Level triggered - 'lvl_trig'
        - Positive edge triggered - 'pos_edg_trig'
        - Negative edge triggered - 'neg_edg_trig'
    """
    def __init__(self, update_func, kind, service_func, enable=True):
        """
        Arguments:
            - update_func: Function which will update this flag
                - Can be None
            - kind: String which tells the kind of this flag
            - service_func: The function to be executed when value=True
                - Can be None
        """
        self.kinds = ['lvl_trig', 'pos_edg_trig', 'neg_edg_trig']
        self.kind = None
        self.update_func = None
        self.flag = None
        self.flag_lag = None
        self.service_func = None
        self.value = False
        self.enable = enable
        self.set_kind(kind)
        self.initialise()
        self.set_update_func(update_func)
        self.set_service_func(service_func)


    def set_kind(self, kind):
        """
        - Check whether the kind type given is correct
        - Raise error if not correct  
        """
        if kind in self.kinds:
            self.kind = kind 
        
        else:
            raise Exception('kind type given to the flag is invalid. \nGiven ' + kind)


    def set_update_func(self, update_func):
        """
        - Check whether the update_func is valid or not
        - Update self.update_func if valid 
        """
        if update_func!=None:
            if callable(update_func):
                self.update_func = update_func

            else:
                raise Exception("The update_func argument is invalid")


    def set_service_func(self, service_func):
        """
        - Check whether the update_func is valid or not
        - Update self.update_func if valid 
        """
        if service_func!=None:
            if callable(service_func):
                self.service_func = service_func

            else:
                raise Exception("The service_func argument is invalid")


    def initialise(self):
        """
        - Initialise the required attributes based on the class type
        """
        if self.kind == 'lvl_trig':
            self.flag = False

        elif self.kind == 'pos_edg_trig':
            self.flag_lag = False
            self.flag = False

        elif self.kind == 'neg_edg_trig':
            self.flag_lag = False
            self.flag = False   


    def update(self):
        """
        Calls the update_flags function of the holder class
        """
        if self.update_func!=None:
            self.update_func()

        if self.kind == 'lvl_trig':
            self.value = self.flag
        elif self.kind == 'pos_edg_trig':
            self.value = ((not self.flag_lag) and self.flag)
        elif self.kind == 'neg_edg_trig':
            self.value = (self.flag_lag and (not self.flag))

        if self.kind!='lvl_trig':
            self.flag_lag = self.flag


    def service(self):
        """
        Execute the service function if any
        """
        if self.service_func!=None and self.value==True:
            self.service_func()


class Flags(object):
    """
    Store all the flags
    """
    def __init__(self):
        self.list = []

    
    def update(self):
        """
        - Update all the flags
        - Execute associated service, if any
        """
        # Update all the flag values
        for flag in self.list:
            if flag.enable:
                flag.update()

        # Execution is done after the update so that
        # it won't affect the state, before all the
        # flags get updated
        for flag in self.list:
            if flag.enable:
                flag.service()


    def add(self, update_func, kind, service_func, enable):
        """
        - Create new flag object 
        - Add it to the list
        - Return the new flag object
        """
        flag = Flag(
            update_func=update_func,
            kind=kind,
            service_func=service_func,
            enable=enable
        )
        self.list.append(flag)
        
        return flag
