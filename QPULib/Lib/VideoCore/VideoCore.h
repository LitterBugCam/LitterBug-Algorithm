#ifdef QPU_MODE
    #pragma once

    // Globals
    extern int mailbox;
    extern int numQPUUsers;

    // Operations
    int getMailbox();
    void enableQPUs();
    void disableQPUs();
#endif
