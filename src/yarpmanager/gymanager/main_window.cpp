/*
 *  Yarp Modules Manager
 *  Copyright: 2011 (C) Robotics, Brain and Cognitive Sciences - Italian Institute of Technology (IIT)
 *  Authors: Ali Paikan <ali.paikan@iit.it>
 * 
 *  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <gtkmm/stock.h>

#include <iostream>
#include <string>

#include "main_window.h"
#include "application_window.h"
#include "icon_res.h"

#define UNIX

#ifdef UNIX
	#include <unistd.h>
	#include <sys/types.h>
	#include <sys/wait.h>
	#include <errno.h>
	#include <string.h>
	#include <sys/types.h>
	#include <dirent.h>
	#include <signal.h>
#endif


using namespace std;

#define WND_DEF_HEIGHT		600
#define WND_DEF_WIDTH		800


MainWindow::MainWindow( yarp::os::Property &config)
{
	m_config = config;

	//fullscreen();
	set_title("Yarp module manager");
	//set_border_width(3);
	set_default_size(WND_DEF_WIDTH, WND_DEF_HEIGHT);

	setupActions();
	setupSignals();
	createWidgets();

	if(config.check("apppath"))
	{
		if(config.find("load_subfolders").asString() == "yes")
			loadRecursiveApplications(config.find("apppath").asString().c_str());
		else
			lazyManager.addApplications(config.find("apppath").asString().c_str());	
	}
	reportErrors();

	syncApplicationList();
	show_all_children();
}


MainWindow::~MainWindow()
{

}


void MainWindow::createWidgets(void)
{

	 set_icon(Gdk::Pixbuf::create_from_data(ymanager_ico.pixel_data, 
						Gdk::COLORSPACE_RGB,
						90,
						8,
						ymanager_ico.height,
						ymanager_ico.width,
						ymanager_ico.bytes_per_pixel*ymanager_ico.width));

	add(m_VBox);

	m_refUIManager = Gtk::UIManager::create();
	m_refUIManager->insert_action_group(m_refActionGroup);
	add_accel_group(m_refUIManager->get_accel_group());

	Glib::ustring ui_info =
		"<ui>"
		" <menubar name='MenuBar'>"
        "    <menu action='FileMenu'>"
        "      <menu action='FileNew'>"
        "        <menuitem action='FileNewApp'/>"
        "        <menuitem action='FileNewMod'/>"
        "      </menu>"
        "      <menuitem action='FileOpen'/>"
        "      <menuitem action='FileClose'/>"
        "      <separator/>"
	    "      <menuitem action='FileSave'/>"
        "      <menuitem action='FileSaveAs'/>"
        "      <separator/>"
	    "      <menuitem action='FileImport'/>"
        "      <separator/>"
        "      <menuitem action='FileQuit'/>"
        "    </menu>"
		"    <menu action='EditMenu'>"
        "      <menuitem action='EditSelAll'/>"
    //  "      <menuitem action='EditSelCon'/>"
		"    </menu>"
		"    <menu action='ManageMenu'>"
        "      <menuitem action='ManageRun'/>"
        "      <menuitem action='ManageStop'/>"
        "      <menuitem action='ManageKill'/>"
        "      <separator/>"
        "      <menuitem action='ManageConnect'/>"
        "      <menuitem action='ManageDisconnect'/>"
        "      <separator/>"
        "      <menuitem action='ManageRefresh'/>"
		"    </menu>"
		"    <menu action='HelpMenu'>"
		"      <menuitem action='HelpOnline'/>"
		"      <menuitem action='HelpAbout'/>"
		"    </menu>"
		" </menubar>"
		" <toolbar name='ToolBar'>"
		"    <toolitem action='FileNew'/>"
		"    <toolitem action='FileOpen'/>"
        "    <separator/>"
		"    <toolitem action='ManageRun'/>"
		"    <toolitem action='ManageStop'/>"
		"    <toolitem action='ManageKill'/>"
		"    <toolitem action='ManageConnect'/>"
		"    <toolitem action='ManageDisconnect'/>"
		"    <toolitem action='ManageRefresh'/>"
	    "    <separator/>"
		"    <toolitem action='HelpOnline'/>"
		" </toolbar>"
		" <popup name='PopupManageModules'>"
        "      <menuitem action='ManageRun'/>"
        "      <menuitem action='ManageStop'/>"
        "      <menuitem action='ManageKill'/>"
        "      <separator/>"
        "      <menuitem action='ManageRefresh'/>"
		" </popup>"
		"</ui>";


#ifdef GLIBMM_EXCEPTIONS_ENABLED
	try
	{
		m_refUIManager->add_ui_from_string(ui_info);
	}
	catch(const Glib::Error& ex)
	{
		std::cerr << "building menus failed: " << ex.what();
	}
#else
	std::auto_ptr<Glib::Error> ex;
	m_refUIManager->add_ui_from_string(ui_info, ex);
	if(ex.get())
	{
  		std::cerr << "building menus failed: " << ex->what();
	}	
#endif //GLIBMM_EXCEPTIONS_ENABLED

	Gtk::Widget* pMenubar = m_refUIManager->get_widget("/MenuBar");
	if(pMenubar)
  		m_VBox.pack_start(*pMenubar, Gtk::PACK_SHRINK);
	Gtk::Widget* pToolbar = m_refUIManager->get_widget("/ToolBar") ;
	if(pToolbar)
	{
		((Gtk::Toolbar*)pToolbar)->set_icon_size(Gtk::IconSize(Gtk::ICON_SIZE_SMALL_TOOLBAR));
		m_VBox.pack_start(*pToolbar, Gtk::PACK_SHRINK);
	}


	m_refCommandBuffer = Gtk::TextBuffer::create();
	m_refCommandBuffer->set_text("Yarp Manager lets you write here whatever you want. Is'n it coool?");
	m_commandView.set_buffer(m_refCommandBuffer);

	m_bottomTab.set_tab_pos(Gtk::POS_BOTTOM);
	m_bottomTab.set_border_width(0);
	m_bottomTab.append_page(m_messageList, "Messages");
	m_bottomTab.append_page(m_commandView, "Notes");

	//m_Notebook.signal_switch_page().connect(sigc::mem_fun(*this,
    //        &ExampleWindow::on_notebook_switch_page) );


	m_mainTab.set_border_width(0);
	m_mainTab.set_scrollable(true);
	m_applicationList.set_border_width(3);	
	m_HPaned.add1(m_applicationList);
	m_HPaned.add2(m_mainTab);
	m_HPaned.set_position(WND_DEF_WIDTH/3);
	
	m_VPaned.add1(m_HPaned);
	m_VPaned.add2(m_bottomTab);
	m_VPaned.set_position(WND_DEF_HEIGHT - WND_DEF_HEIGHT/3);
	m_VPaned.set_size_request(-1, 300);
	m_VBox.pack_start(m_VPaned);
	m_VBox.pack_start(m_Statusbar, Gtk::PACK_SHRINK);
	

}


void MainWindow::setupSignals(void)
{
	//Connect signal:
	
  	m_applicationList.getTreeView()->signal_row_activated().connect(sigc::mem_fun(*this,
              			&MainWindow::onAppListRowActivated) );

	m_mainTab.signal_switch_page().connect(sigc::mem_fun(*this,
            &MainWindow::onNotebookSwitchPage) );

	signal_delete_event().connect(sigc::mem_fun(*this, 
			&MainWindow::onDeleteEvent));

	
//	signal_expose_event().connect(sigc::mem_fun(*this,
//            &MainWindow::onExposeEvent) );	
}

void MainWindow::setupActions(void)
{

	//Create actions for menus and toolbars:
  	m_refActionGroup = Gtk::ActionGroup::create();

	//File|New sub menu:
	m_refActionGroup->add(Gtk::Action::create("FileMenu", "File"));
	m_refActionGroup->add(Gtk::Action::create("FileNew", Gtk::Stock::NEW));
	m_refActionGroup->add(Gtk::Action::create("FileNewApp",
				Gtk::Stock::NEW, "New _Application", "Create a new Application"),
				sigc::mem_fun(*this, &MainWindow::onMenuFileNewApp));
	m_refActionGroup->add(Gtk::Action::create("FileNewMod",
				Gtk::Stock::NEW, "New _Module", "Create a new Module"),
				sigc::mem_fun(*this, &MainWindow::onMenuFileNewMod));
	m_refActionGroup->add( Gtk::Action::create("FileOpen", Gtk::Stock::OPEN),
						sigc::mem_fun(*this, &MainWindow::onMenuFileOpen) );
	m_refActionGroup->add( Gtk::Action::create("FileClose", Gtk::Stock::CLOSE),
						sigc::mem_fun(*this, &MainWindow::onMenuFileClose) );

	m_refActionGroup->add( Gtk::Action::create("FileSave", Gtk::Stock::SAVE),
						sigc::mem_fun(*this, &MainWindow::onMenuFileSave) );
	m_refActionGroup->add( Gtk::Action::create("FileSaveAs", Gtk::Stock::SAVE_AS),
						sigc::mem_fun(*this, &MainWindow::onMenuFileSaveAs) );
	m_refActionGroup->add( Gtk::Action::create("FileImport", "_Import...", "Import xml files"),
						sigc::mem_fun(*this, &MainWindow::onMenuFileImport) );

	m_refActionGroup->add(Gtk::Action::create("FileQuit", Gtk::Stock::QUIT),
		  sigc::mem_fun(*this, &MainWindow::onMenuFileQuit));
	
	//Edit menu:
	m_refActionGroup->add( Gtk::Action::create("EditMenu", "Edit") );
	m_refActionGroup->add( Gtk::Action::create("EditSelAll", Gtk::Stock::SELECT_ALL, "Select All", "Select all"),
						sigc::mem_fun(*this, &MainWindow::onMenuEditSellAll) );
//	m_refActionGroup->add( Gtk::Action::create("EditSelCon", Gtk::Stock::SELECT_ALL, "_Select All Mod", "Select all Modules"),
//						sigc::mem_fun(*this, &MainWindow::onMenuEditSellAllMod) );

	//Manage menu:
	m_refActionGroup->add( Gtk::Action::create("ManageMenu", "Manage") );
	m_refActionGroup->add( Gtk::Action::create("ManageRun", Gtk::Stock::EXECUTE, "_Run", "Run Application"),
							sigc::mem_fun(*this, &MainWindow::onMenuManageRun) );
	m_refActionGroup->add( Gtk::Action::create("ManageStop", Gtk::Stock::CLOSE ,"_Stop", "Stop Application"),
							sigc::mem_fun(*this, &MainWindow::onMenuManageStop) );
	m_refActionGroup->add( Gtk::Action::create("ManageKill", Gtk::Stock::STOP,"_Kill", "Kill Application"),
							sigc::mem_fun(*this, &MainWindow::onMenuManageKill) );
	m_refActionGroup->add( Gtk::Action::create("ManageConnect", Gtk::Stock::CONNECT, "_Connect", "Connect links"),
							sigc::mem_fun(*this, &MainWindow::onMenuManageConnect) );
	m_refActionGroup->add( Gtk::Action::create("ManageDisconnect", Gtk::Stock::DISCONNECT, "_Disconnect", "Disconnect links"),
							sigc::mem_fun(*this, &MainWindow::onMenuManageDisconnect) );
	m_refActionGroup->add( Gtk::Action::create("ManageRefresh", Gtk::Stock::REFRESH, "Re_fresh Status", "Refresh Modules/connections Status"),
							sigc::mem_fun(*this, &MainWindow::onMenuManageRefresh) );

	//Help menu:
	m_refActionGroup->add( Gtk::Action::create("HelpMenu", "Help") );
	m_refActionGroup->add( Gtk::Action::create("HelpOnline", Gtk::Stock::HELP),
							sigc::mem_fun(*this, &MainWindow::onMenuHelpOnlineHelp) );
	m_refActionGroup->add( Gtk::Action::create("HelpAbout", Gtk::Stock::ABOUT),
							sigc::mem_fun(*this, &MainWindow::onMenuHelpAbout) );
	
	// initial sensitivity
	m_refActionGroup->get_action("FileNew")->set_sensitive(false);
	m_refActionGroup->get_action("FileClose")->set_sensitive(false);
	m_refActionGroup->get_action("FileSave")->set_sensitive(false);
	m_refActionGroup->get_action("FileSaveAs")->set_sensitive(false);
	m_refActionGroup->get_action("EditSelAll")->set_sensitive(false);

	m_refActionGroup->get_action("ManageRun")->set_sensitive(false);
	m_refActionGroup->get_action("ManageStop")->set_sensitive(false);
	m_refActionGroup->get_action("ManageKill")->set_sensitive(false);
	m_refActionGroup->get_action("ManageConnect")->set_sensitive(false);
	m_refActionGroup->get_action("ManageDisconnect")->set_sensitive(false);
	m_refActionGroup->get_action("ManageRefresh")->set_sensitive(false);

}


void MainWindow::reportErrors(void)
{
	ErrorLogger* logger  = ErrorLogger::Instance();	
	if(logger->errorCount() || logger->warningCount())
	{
		const char* err;
		while((err=logger->getLastError()))
			m_messageList.addError(err);

		while((err=logger->getLastWarning()))
			m_messageList.addWarning(err);
	}	
}

bool MainWindow::loadRecursiveApplications(const char* szPath)
{
#ifdef UNIX

	string strPath = szPath;
	if((strPath.rfind("/")==string::npos) || 
			(strPath.rfind("/")!=strPath.size()-1))
			strPath = strPath + string("/");

	DIR *dir;
	struct dirent *entry;
	if ((dir = opendir(strPath.c_str())) == NULL)
		return false;

	lazyManager.addApplications(strPath.c_str());

	while((entry = readdir(dir)))
	{
		if((string(entry->d_name) != string(".")) 
		&& (string(entry->d_name) != string("..")))
		{
			string name = strPath + string(entry->d_name);
			loadRecursiveApplications(name.c_str());
		}
	}
	closedir(dir);
#endif
	return true;
}



void MainWindow::syncApplicationList(void)
{
	KnowledgeBase* kb = lazyManager.getKnowledgeBase();
	ApplicaitonPContainer apps =  kb->getApplications();
	unsigned int cnt = 0;
	for(ApplicationPIterator itr=apps.begin(); itr!=apps.end(); itr++)
	{
		cnt++;
		//m_applicationList.addApplication((*itr)->getName());
		m_applicationList.addApplication((*itr));
	}
	
	if(cnt)
	{
		ostringstream msg;
		msg<<cnt<<" "<<"applications are loaded successfully.";
		m_Statusbar.push(msg.str().c_str());
	}
	else
		m_Statusbar.push("No application is loaded!");
}


bool MainWindow::onDeleteEvent(GdkEventAny* event)
{
	return !safeExit();
}

void MainWindow::onMenuFileQuit()
{
	if(safeExit())
		hide();
}


bool MainWindow::safeExit(void)
{
	bool bSafe = true;	
	for(int i=0; i<m_mainTab.get_n_pages(); i++)
	{
		ApplicationWindow* appWnd = 
				(ApplicationWindow*) m_mainTab.get_nth_page(i);
		bSafe &= appWnd->onClose();
		if(!bSafe)
			return false;
	}
	return bSafe;
}


void MainWindow::onMenuFileNewApp() { }
void MainWindow::onMenuFileNewMod() { }

void MainWindow::onMenuFileClose()
{
	int page_num = m_mainTab.get_current_page();
	ApplicationWindow* appWnd = (ApplicationWindow*) m_mainTab.get_nth_page(page_num);
	if(appWnd)
	{
		if(appWnd->onClose())
			m_mainTab.remove_page(page_num);
	}

	if(!m_mainTab.get_n_pages())
	{
		m_Statusbar.push("No application is loaded");
		m_refActionGroup->get_action("FileClose")->set_sensitive(false);
		m_refActionGroup->get_action("ManageRun")->set_sensitive(false);
		m_refActionGroup->get_action("ManageStop")->set_sensitive(false);
		m_refActionGroup->get_action("ManageKill")->set_sensitive(false);
		m_refActionGroup->get_action("ManageConnect")->set_sensitive(false);
		m_refActionGroup->get_action("ManageDisconnect")->set_sensitive(false);
		m_refActionGroup->get_action("ManageRefresh")->set_sensitive(false);
	}
}

void MainWindow::onMenuFileOpen() 
{
	Gtk::FileChooserDialog dialog("Please choose a folder");
  	dialog.set_transient_for(*this);

	//Add response buttons the the dialog:
	dialog.add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
	dialog.add_button("Open", Gtk::RESPONSE_OK);


	//Add filters, so that only certain file types can be selected:
	Gtk::FileFilter filter_app;
	filter_app.set_name("Application description files");
	filter_app.add_mime_type("text/xml");
	dialog.add_filter(filter_app);

	Gtk::FileFilter filter_mod;
	filter_mod.set_name("Modules description files");
	filter_mod.add_mime_type("text/xml");
	dialog.add_filter(filter_mod);

	Gtk::FileFilter filter_any;
	filter_any.set_name("Any files");
	filter_any.add_pattern("*");
	dialog.add_filter(filter_any);

	if(dialog.run() == Gtk::RESPONSE_OK)
	{
		string fname = dialog.get_filename();
		if(lazyManager.addApplication(fname.c_str()))
		{
			syncApplicationList();
		}
		else if(lazyManager.addModule(fname.c_str()))
		{
			ostringstream msg;
			msg<<"Module "<<fname<<" is added.";
			m_Statusbar.push(msg.str());
		}
		reportErrors();
	}

}

void MainWindow::onMenuFileSave() { }
void MainWindow::onMenuFileSaveAs() { }

void MainWindow::onMenuFileImport() 
{
	Gtk::FileChooserDialog dialog("Please choose a folder", Gtk::FILE_CHOOSER_ACTION_SELECT_FOLDER);
  	dialog.set_transient_for(*this);

	//Add response buttons the the dialog:
	dialog.add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
	dialog.add_button("Import", Gtk::RESPONSE_OK);

	if(dialog.run() == Gtk::RESPONSE_OK)
	{
		string fname = dialog.get_filename();
		
		if(m_config.find("load_subfolders").asString() == "yes")
		{
			if(loadRecursiveApplications(fname.c_str()))
				syncApplicationList();
		}
		else
		{
			if(lazyManager.addApplications(fname.c_str()))
				syncApplicationList();
		}

		if(lazyManager.addModules(fname.c_str()))
		{
			ostringstream msg;
			msg<<"Modules from "<<fname<<" are added.";
			m_Statusbar.push(msg.str());
		}
		reportErrors();
	}

}

void MainWindow::onMenuHelpOnlineHelp() 
{
	onMenuHelpAbout();
}

void MainWindow::onMenuHelpAbout()
{

	Gtk::AboutDialog dialog; 
	dialog.set_program_name("YARP Manager");
	dialog.set_version("1.0");
	dialog.set_copyright(
			"2011 (C) Robotics, Brain and Cognitive Sciences\n"
   		    "Italian Institute of Technology (IIT)");
	dialog.set_license("Released under the terms of the LGPLv2.1 or later.");
	dialog.set_website("http://eris.liralab.it/yarp");
	std::vector<Glib::ustring> authors;
	authors.push_back("Ali Paikan <ali.paikan@iit.it>");
	dialog.set_authors(authors);
	dialog.set_logo(Gdk::Pixbuf::create_from_data(ymanager_ico.pixel_data, 
						Gdk::COLORSPACE_RGB,
						90,
						8,
						ymanager_ico.height,
						ymanager_ico.width,
						ymanager_ico.bytes_per_pixel*ymanager_ico.width));

	dialog.run();
}

void MainWindow::onMenuManageRun() 
{
	int page_num = m_mainTab.get_current_page();
	ApplicationWindow* appWnd = (ApplicationWindow*) m_mainTab.get_nth_page(page_num);
	if(appWnd)
		appWnd->onRun();
}

void MainWindow::onMenuManageStop() 
{
	int page_num = m_mainTab.get_current_page();
	ApplicationWindow* appWnd = (ApplicationWindow*) m_mainTab.get_nth_page(page_num);
	if(appWnd)
		appWnd->onStop();
}

void MainWindow::onMenuManageKill()
{
	int page_num = m_mainTab.get_current_page();
	ApplicationWindow* appWnd = (ApplicationWindow*) m_mainTab.get_nth_page(page_num);
	if(appWnd)
		appWnd->onKill();
}

void MainWindow::onMenuManageConnect() 
{
	int page_num = m_mainTab.get_current_page();
	ApplicationWindow* appWnd = (ApplicationWindow*) m_mainTab.get_nth_page(page_num);
	if(appWnd)
		appWnd->onConnect();
}


void MainWindow::onMenuManageDisconnect() 
{
	int page_num = m_mainTab.get_current_page();
	ApplicationWindow* appWnd = (ApplicationWindow*) m_mainTab.get_nth_page(page_num);
	if(appWnd)
		appWnd->onDisconnect();
}

void MainWindow::onMenuManageRefresh() 
{
	int page_num = m_mainTab.get_current_page();
	ApplicationWindow* appWnd = (ApplicationWindow*) m_mainTab.get_nth_page(page_num);
	if(appWnd)
		appWnd->onRefresh();
}


void MainWindow::onMenuEditSellAll() 
{
	int page_num = m_mainTab.get_current_page();
	ApplicationWindow* appWnd = (ApplicationWindow*) m_mainTab.get_nth_page(page_num);
	if(appWnd)
		appWnd->onSelectAll();
}


void MainWindow::onAppListRowActivated(const Gtk::TreeModel::Path& path, 
			Gtk::TreeViewColumn* column)
{
	Gtk::TreeModel::iterator iter = m_applicationList.m_refTreeModel->get_iter(path);
	
	if(iter && ((*iter)[m_applicationList.m_appColumns.m_col_type] == APPLICATION)) 
  	{
		Gtk::TreeModel::Row row = *iter;
		Glib::ustring name = row[m_applicationList.m_appColumns.m_col_name];
		
		int page_num = -1;
		for(int i=0; i<m_mainTab.get_n_pages(); i++)
		{
			Glib::ustring pageName = m_mainTab.get_tab_label_text(*m_mainTab.get_nth_page(i));
			if(pageName == name)
			{
				page_num = i;
				break;
			}
		}

		if(page_num>=0)
			m_mainTab.set_current_page(page_num);
		else
		{
			ApplicationWindow* pAppWnd = Gtk::manage(new ApplicationWindow(name.c_str(), 
											&lazyManager, &m_config, this));
			m_mainTab.append_page(*pAppWnd, name);
			m_mainTab.set_tab_reorderable(*pAppWnd);
			m_mainTab.show_all_children();
			m_mainTab.set_current_page(m_mainTab.get_n_pages()-1);
		}
		
		m_refActionGroup->get_action("FileClose")->set_sensitive(true);
		m_refActionGroup->get_action("ManageRun")->set_sensitive(true);
		m_refActionGroup->get_action("ManageStop")->set_sensitive(true);
		m_refActionGroup->get_action("ManageKill")->set_sensitive(true);
		m_refActionGroup->get_action("ManageConnect")->set_sensitive(true);
		m_refActionGroup->get_action("ManageDisconnect")->set_sensitive(true);
		m_refActionGroup->get_action("ManageRefresh")->set_sensitive(true);

	}
	

}

void MainWindow::onNotebookSwitchPage(GtkNotebookPage* page, guint page_num)
{
	//ApplicationWindow* appWnd = (ApplicationWindow*) m_mainTab.get_nth_page(page_num);
	Glib::ustring name = m_mainTab.get_tab_label_text(*m_mainTab.get_nth_page(page_num));
	ostringstream msg;
	msg<<"Current application: "<<name.c_str();
	m_Statusbar.push(msg.str());
}


bool MainWindow::onExposeEvent( GdkEventExpose* event)
{
//	cout<<get_height()<<endl;
	m_VPaned.set_position(get_height() - get_height()/3);

	return Gtk::Window::on_expose_event(event);	
}



