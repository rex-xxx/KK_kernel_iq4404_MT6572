/*
 * Copyright (c) 1998-2011 Erez Zadok
 * Copyright (c) 2009	   Shrikar Archak
 * Copyright (c) 2003-2011 Stony Brook University
 * Copyright (c) 2003-2011 The Research Foundation of SUNY
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "sdcardfs.h"
#include <linux/module.h>
#include <linux/types.h>
#include <linux/parser.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include "../internal.h"

enum {
	Opt_uid, 
	Opt_gid, 
	Opt_debug,
	Opt_derive,
	Opt_write_gid,
	Opt_err,
};

static const match_table_t sdcardfs_tokens = {
	{Opt_uid, "uid=%u"},
	{Opt_gid, "gid=%u"},
	{Opt_debug, "debug"},
	{Opt_derive, "derive=%s"},
	{Opt_write_gid, "write_gid=%u"},
	{Opt_err, NULL}
};

static void create_proc_intf(void);

static int parse_options(struct super_block *sb, char *options, int silent, 
				int *debug, struct sdcardfs_mount_options *opts)
{
	char *p;
	substring_t args[MAX_OPT_ARGS];
	int option;
	char *derive;

	SDFS_DBG("options='%s' \n", options);			
	/* by default, we use AID_MEDIA_RW as uid, gid */
	opts->fs_low_uid = AID_MEDIA_RW;
	opts->fs_low_gid = AID_MEDIA_RW;
	opts->derive = DERIVE_UNIFIED;
	opts->write_gid = AID_SDCARD_RW;

	*debug = 0;

	if (!options)
		return 0;

	while ((p = strsep(&options, ",")) != NULL) {
		int token;
		
		SDFS_DBG("p=%s \n", p);
		if (!*p)
			continue;

		token = match_token(p, sdcardfs_tokens, args);
		
		switch (token) {

		//printk( KERN_INFO "sdcardfs : options - token:%d\n", token);
		case Opt_debug:
			*debug = 1;
			break;
		case Opt_uid:
			if (match_int(&args[0], &option))
				return 0;
			opts->fs_low_uid = option;
			break;
		case Opt_gid:
			if (match_int(&args[0], &option))
				return 0;
			opts->fs_low_gid = option;
			break;

		case Opt_derive:
			//printk( KERN_INFO "sdcardfs : options - args:'%s'\n", &args[0]);
			derive = match_strdup(&args[0]);

			if (!strcmp("legacy", derive)) {
				opts->derive = DERIVE_LEGACY;				
			}
			if (!strcmp("unified", derive)) {
				opts->derive = DERIVE_UNIFIED;				
			}			
			kfree(derive);
			break;
		case Opt_write_gid:
			if (match_int(&args[0], &option))
				return 0;
			opts->write_gid = option;
			break;


		/* unknown option */
		default:
			if (!silent) {
			printk( KERN_ERR "Unrecognized mount option \"%s\" "
						"or missing value", p);
			}
			return -EINVAL;
		}
	}

	if (*debug) {
		printk( KERN_INFO "sdcardfs : options - debug:%d\n", *debug);
		printk( KERN_INFO "sdcardfs : options - uid:%d\n", 
							opts->fs_low_uid);
		printk( KERN_INFO "sdcardfs : options - gid:%d\n", 
							opts->fs_low_gid);
		printk( KERN_INFO "sdcardfs : options - derive:%d\n", 
								opts->derive);
		printk( KERN_INFO "sdcardfs : options - write_gid:%d\n", 
										opts->write_gid);

	}

	return 0;
}

int open_flags_to_access_mode(int open_flags) {
    if ((open_flags & O_ACCMODE) == O_RDONLY) {
        return R_OK;
    } else if ((open_flags & O_ACCMODE) == O_WRONLY) {
        return W_OK;
    } else {
        /* Probably O_RDRW, but treat as default to be safe */
        return R_OK | W_OK;
    }
}

/* Kernel has already enforced everything we returned through
 * derive_permissions_locked(), so this is used to lock down access
 * even further, such as enforcing that apps hold sdcard_rw. */
bool check_caller_access_to_name(struct dentry *dentry, int mode, bool has_rw) {
    struct dentry *dentry_parent = dget_parent(dentry);
	struct inode *parent = dentry_parent->d_inode;
	struct sdcardfs_inode_info *info_parent = SDCARDFS_I(parent);
	struct sdcardfs_sb_info *sb_info = SDCARDFS_SB(parent->i_sb);

    uid_t uid = current_fsuid();  
	const char* name = dentry->d_name.name;
    bool rtn = true;

    /* Always block security-sensitive files at root */
    if (info_parent && info_parent->perm == PERM_ROOT) {
        if (!strcasecmp(name, "autorun.inf")
                || !strcasecmp(name, ".android_secure")
                || !strcasecmp(name, "android_secure")) {
            rtn = false;
            goto out;           
        }
    }

    /* No additional permissions enforcement */
    if (sb_info->options.derive == DERIVE_NONE) {
         rtn = true;
         goto out;   
    }

    /* Root always has access; access for any other UIDs should always
     * be controlled through packages.list. */
    if (uid == 0) {
		rtn = true;
	    goto out;
    }

    /* If asking to write, verify that caller either owns the
     * parent or holds sdcard_rw. */
    if (mode & W_OK) {
        if (parent && current_fsuid() == parent->i_uid) {
              rtn = true;
              goto out;
        }
		rtn = has_rw;
		goto out;
    }
out:

	dput(dentry_parent);
    /* No extra permissions to enforce */

	SDFS_DBG("dentry='%s' parent='%s', rtn=%d \n",dentry->d_name.name,dentry->d_parent->d_name.name, rtn);
    return rtn;
}

bool check_caller_access_to_node(struct dentry *dentry, int mode, bool has_rw) {
    return check_caller_access_to_name(dentry, mode, has_rw);
}


int fs_prepare_dir(const char* path, mode_t mode, uid_t uid, gid_t gid) {
	int err = 0;
	struct path path_to_make;

	SDFS_DBG("path='%s', mode=%o, uid=%d, gid=%d \n", path, mode, uid, gid);
	err = kern_path(path, LOOKUP_FOLLOW | LOOKUP_DIRECTORY,
				&path_to_make);
	if (err) {
			printk(KERN_ERR "sdcardfs: error accessing "
				   "lower directory '%s'\n", path);
			goto out_free;
		}

	sdcardfs_mkdir(path_to_make.dentry->d_inode, path_to_make.dentry, mode);

out_free:
	path_put(&path_to_make);

}

void derive_permissions_locked_for_root_node(struct inode *inode)
{   
	struct sdcardfs_sb_info *sb_info = SDCARDFS_SB(inode->i_sb);  
	struct sdcardfs_inode_info *info = SDCARDFS_I(inode);
	inode->i_uid = AID_ROOT;

	SDFS_DBG("derive=%d \n", sb_info->options.derive);

	switch (sb_info->options.derive) {
	   case DERIVE_NONE:
		   /* Traditional behavior that treats entire device as being accessible
			* to sdcard_rw, and no permissions are derived. */
		   info->perm = PERM_ROOT;
		   //inode->i_mode = 775;
		   set_mode(inode, 0775);
		   inode->i_gid = AID_SDCARD_RW;
		   break;
	   case DERIVE_LEGACY:
		   /* Legacy behavior used to support internal multiuser layout which
			* places user_id at the top directory level, with the actual roots
			* just below that. Shared OBB path is also at top level. */
		   info->perm = PERM_LEGACY_PRE_ROOT;
		   //inode->i_mode = 771;
		   set_mode(inode, 0771);
		   inode->i_gid = AID_SDCARD_R;
		   snprintf(sb_info->obbpath, sizeof(sb_info->obbpath), "%s/obb", sb_info->source_path);
		   //fs_prepare_dir(sb_info->obbpath, 0775, AID_MEDIA_RW, AID_MEDIA_RW);
		   break;
	   case DERIVE_UNIFIED:
		   /* Unified multiuser layout which places secondary user_id under
			* /Android/user and shared OBB path under /Android/obb. */
		   info->perm = PERM_ROOT;
		   //inode->i_mode = 771;
		   set_mode(inode, 0771);
		   inode->i_gid = AID_SDCARD_R; 
		   snprintf(sb_info->obbpath, sizeof(sb_info->obbpath), "%s/Android/obb", sb_info->source_path);
		   break;
	   }		
}

void derive_permissions_locked(struct dentry *dentry) 
{
	struct inode *inode = dentry->d_inode;
	struct sdcardfs_inode_info *info;
	struct dentry *dentry_parent = dget_parent(dentry);
	struct inode *parent = dentry_parent->d_inode;
	struct sdcardfs_inode_info *info_parent = SDCARDFS_I(parent);
	const char *name = dentry->d_name.name;
	struct sdcardfs_sb_info *sb_info = SDCARDFS_SB(parent->i_sb);
	appid_t appid;

    if (!inode) {
		SDFS_ERR("inode is NULL, name='%s' \n", name);
		dput(dentry_parent);
        return;
    }
	if (inode && is_bad_inode(inode)) {
		SDFS_ERR("inode is bad, name='%s' \n", name);
		dput(dentry_parent);
		return ;
    }

	info = SDCARDFS_I(inode);
	SDFS_DBG("name='%s' \n", dentry->d_name.name);
	/* By default, each node inherits from its parent */
	info->perm = PERM_INHERIT;
	info->userid = info_parent->userid;
	inode->i_uid = parent->i_uid;
	inode->i_gid = parent->i_gid;
	//inode->i_mode = parent->i_mode & (~S_IFMT);
	set_mode(inode, parent->i_mode);

	if (sb_info->options.derive == DERIVE_NONE) {
		dput(dentry_parent);
		return;
	}

	SDFS_DBG("info_parent->perm=%d \n", info_parent->perm);

	/* Derive custom permissions based on parent and current node */
	switch (info_parent->perm) {
		case PERM_INHERIT:      
			if (!strcasecmp(name, "/")) {
				//SDFS_DBG("root \n");
				derive_permissions_locked_for_root_node(inode);
			}
			else {
				/* Already inherited above */
				//SDFS_DBG("Not root \n");
			}
			break;
		case PERM_LEGACY_PRE_ROOT:
			/* Legacy internal layout places users at top level */
			info->perm = PERM_ROOT;
			info->userid = simple_strtol(name, NULL, 10);
			break;
		case PERM_ROOT:
			/* Assume masked off by default. */
			//inode->i_mode = 770;
			set_mode(inode, 0770);

			if (!strcasecmp(name, "Android")) {
				/* App-specific directories inside; let anyone traverse */
				info->perm = PERM_ANDROID;
				//inode->i_mode = 771;
				set_mode(inode, 0771);
			//} else if (fuse->split_perms) {
			} else if (false) {
				if (!strcasecmp(name, "DCIM")
				|| !strcasecmp(name, "Pictures")) {
					inode->i_gid = AID_SDCARD_PICS;
				} else if (!strcasecmp(name, "Alarms")
					|| !strcasecmp(name, "Movies")
					|| !strcasecmp(name, "Music")
					|| !strcasecmp(name, "Notifications")
					|| !strcasecmp(name, "Podcasts")
					|| !strcasecmp(name, "Ringtones")) {
					
					inode->i_gid = AID_SDCARD_AV;
				}
			}
			break;
		case PERM_ANDROID:
			if (!strcasecmp(name, "data")) {
				/* App-specific directories inside; let anyone traverse */
				info->perm = PERM_ANDROID_DATA;
				//inode->i_mode = 771;
				set_mode(inode, 0771);
			} else if (!strcasecmp(name, "obb")) {
				/* App-specific directories inside; let anyone traverse */
				info->perm = PERM_ANDROID_OBB;
				//inode->i_mode = 771;
				set_mode(inode, 0771);
				/* Single OBB directory is always shared */
				//node->graft_path = fuse->obbpath;
				//node->graft_pathlen = strlen(fuse->obbpath);
			} else if (!strcasecmp(name, "user")) {
				/* User directories must only be accessible to system, protected
				* by sdcard_all. Zygote will bind mount the appropriate user-
				* specific path. */
				info->perm = PERM_ANDROID_USER;
				inode->i_gid = AID_SDCARD_ALL;
				//inode->i_mode = 770;
				set_mode(inode, 0770);
			}
			break;
			case PERM_ANDROID_DATA:
			case PERM_ANDROID_OBB:
				appid = get_pkg_uid(name);
				if (appid != 0) {
					inode->i_uid = multiuser_get_uid(info_parent->userid, appid);
				}
				//inode->i_mode = 770;
				set_mode(inode, 0770);
				break;
		case PERM_ANDROID_USER:
			/* Root of a secondary user */
			info->perm = PERM_ROOT;
			info->userid = simple_strtol(name, NULL, 10);
			inode->i_gid = AID_SDCARD_R;
			//inode->i_mode = 771;
			set_mode(inode, 0771);
			break;
	}

	dput(dentry_parent);

	SDFS_DBG("info->perm=%d, name='%s', i_uid=%d, i_gid=%d, i_mode=%o \n", 
		      info->perm, name, inode->i_uid, inode->i_gid, inode->i_mode);
}

/*
 * our custom d_alloc_root work-alike
 *
 * we can't use d_alloc_root if we want to use our own interpose function
 * unchanged, so we simply call our own "fake" d_alloc_root
 */
static struct dentry *sdcardfs_d_alloc_root(struct super_block *sb)
{
	struct dentry *ret = NULL;

	if (sb) {
		static const struct qstr name = {
			.name = "/",
			.len = 1
		};

		ret = __d_alloc(sb, &name);
		if (ret)
			d_set_d_op(ret, &sdcardfs_dops);
	}
	return ret;
}

/*
 * There is no need to lock the sdcardfs_super_info's rwsem as there is no
 * way anyone can have a reference to the superblock at this point in time.
 */
static int sdcardfs_read_super(struct super_block *sb, const char *dev_name, 
						void *raw_data, int silent)
{
	int err = 0;
	int debug;
	struct super_block *lower_sb;
	struct path lower_path;
	struct sdcardfs_sb_info *sb_info;

	if (!dev_name) {
		printk(KERN_ERR
		       "sdcardfs: read_super: missing dev_name argument\n");
		err = -EINVAL;
		goto out;
	}

	SDFS_DBG("dev_name -> %s\n", dev_name);
	SDFS_DBG("raw_data -> %s\n", (char *)raw_data);
	printk(KERN_INFO "sdcardfs: dev_name -> %s\n", dev_name);
	printk(KERN_INFO "sdcardfs: options -> %s\n", (char *)raw_data);

	/* parse lower path */
	err = kern_path(dev_name, LOOKUP_FOLLOW | LOOKUP_DIRECTORY,
			&lower_path);
	if (err) {
		printk(KERN_ERR	"sdcardfs: error accessing "
		       "lower directory '%s'\n", dev_name);
		goto out;
	}

	/* allocate superblock private data */
	sb->s_fs_info = kzalloc(sizeof(struct sdcardfs_sb_info), GFP_KERNEL);
	if (!SDCARDFS_SB(sb)) {
		printk(KERN_CRIT "sdcardfs: read_super: out of memory\n");
		err = -ENOMEM;
		goto out_free;
	}

	/* setup fs_uid and fs_gid for FAT emulation : wjlee */
	sb_info = sb->s_fs_info;
	sb_info->fs_uid = AID_ROOT;
	sb_info->fs_gid = AID_SDCARD_R; 
	sprintf(sb_info->source_path, "%s", dev_name);


	/* parse options */
	err = parse_options(sb, raw_data, silent, &debug, &sb_info->options);
	if (err) {
		printk(KERN_ERR	"sdcardfs: invalid options\n");
		goto out_freesbi;
	}



	/* set the lower superblock field of upper superblock */
	lower_sb = lower_path.dentry->d_sb;
	atomic_inc(&lower_sb->s_active);
	sdcardfs_set_lower_super(sb, lower_sb);

	/* inherit maxbytes from lower file system */
	sb->s_maxbytes = lower_sb->s_maxbytes;

	/*
	 * Our c/m/atime granularity is 1 ns because we may stack on file
	 * systems whose granularity is as good.
	 */
	sb->s_time_gran = 1;

	sb->s_op = &sdcardfs_sops;

	/* see comment next to the definition of sdcardfs_d_alloc_root */
	sb->s_root = sdcardfs_d_alloc_root(sb);
	if (!sb->s_root) {
		err = -ENOMEM;
		goto out_sput;
	}

	/* link the upper and lower dentries */
	sb->s_root->d_fsdata = NULL;
	err = new_dentry_private_data(sb->s_root);
	if (err)
		goto out_freeroot;

	/* set the lower dentries for s_root */
	sdcardfs_set_lower_path(sb->s_root, &lower_path);

	/* call interpose to create the upper level inode */
	err = sdcardfs_interpose(sb->s_root, sb, &lower_path);
	if (!err) {
		if (!silent)
			printk(KERN_INFO
			       "sdcardfs: mounted on top of %s type %s\n",
			       dev_name, lower_sb->s_type->name);
		goto out;
	}
	/* else error: fall through */

	free_dentry_private_data(sb->s_root);
out_freeroot:
	dput(sb->s_root);
out_sput:
	/* drop refs we took earlier */
	atomic_dec(&lower_sb->s_active);
out_freesbi:
	kfree(SDCARDFS_SB(sb));
	sb->s_fs_info = NULL;
out_free:
	path_put(&lower_path);

out:
	return err;
}

/* A feature which supports mount_nodev() with options */
static struct dentry *mount_nodev_with_options(struct file_system_type *fs_type,
        int flags, const char *dev_name, void *data,
        int (*fill_super)(struct super_block *, const char *, void *, int))

{
	int error;
	struct super_block *s = sget(fs_type, NULL, set_anon_super, NULL);

	if (IS_ERR(s))
		return ERR_CAST(s);

	s->s_flags = flags;

	error = fill_super(s, dev_name, data, flags & MS_SILENT ? 1 : 0);
	if (error) {
		deactivate_locked_super(s);
		return ERR_PTR(error);
	}
	s->s_flags |= MS_ACTIVE;
	return dget(s->s_root);
}

struct dentry *sdcardfs_mount(struct file_system_type *fs_type, int flags,
			    const char *dev_name, void *raw_data)
{
	/* 
	 * dev_name is a lower_path_name,
	 * raw_data is a option string.
	 */
	return mount_nodev_with_options(fs_type, flags, dev_name,
					raw_data, sdcardfs_read_super);
}

static struct file_system_type sdcardfs_fs_type = {
	.owner		= THIS_MODULE,
	.name		= SDCARDFS_NAME,
	.mount		= sdcardfs_mount,
	.kill_sb	= generic_shutdown_super,
	.fs_flags	= FS_REVAL_DOT,
};

static int __init init_sdcardfs_fs(void)
{
	int err;

	pr_info("Registering sdcardfs " SDCARDFS_VERSION "\n");

	err = sdcardfs_init_inode_cache();
	if (err)
		goto out;
	err = sdcardfs_init_dentry_cache();
	if (err)
		goto out;
	err = register_filesystem(&sdcardfs_fs_type);
out:
	if (err) {
		sdcardfs_destroy_inode_cache();
		sdcardfs_destroy_dentry_cache();
	}
	
	create_proc_intf();
	return err;
}

static void __exit exit_sdcardfs_fs(void)
{
	sdcardfs_destroy_inode_cache();
	sdcardfs_destroy_dentry_cache();
	unregister_filesystem(&sdcardfs_fs_type);
	pr_info("Completed sdcardfs module unload\n");
}



userid_t multiuser_get_user_id(uid_t uid) {
    return uid / MULTIUSER_APP_PER_USER_RANGE;
}

appid_t multiuser_get_app_id(uid_t uid) {
    return uid % MULTIUSER_APP_PER_USER_RANGE;
}

uid_t multiuser_get_uid(userid_t userId, appid_t appId) {
    return userId * MULTIUSER_APP_PER_USER_RANGE + (appId % MULTIUSER_APP_PER_USER_RANGE);
}

DEFINE_SPINLOCK(listlock);

LIST_HEAD(g_pkg_list);
LIST_HEAD(g_rw_uid_main_list);
LIST_HEAD(g_rw_uid_sub_list);

static struct kmem_cache *pkg_uid_cachep;
static struct kmem_cache *rw_uid_cachep;
bool g_sdcardfs_debug;

#define MAX_NAME_LEN	511

struct pkg_uid{
	struct list_head 	pkg_list;
	unsigned int		uid;
	char			pkg_name[MAX_NAME_LEN + 1];
};

struct rw_uid{
	struct list_head 	rw_uid_list;
	unsigned int		app_uid;
};


static inline struct rw_uid *uid_main_entry_alloc(void)
{
	return kmem_cache_alloc(rw_uid_cachep, GFP_NOFS);
}

static inline void uid_main_entry_free(struct rw_uid *uid)
{
	kmem_cache_free(rw_uid_cachep, uid);
}

static inline struct rw_uid *uid_sub_entry_alloc(void)
{
	return kmem_cache_alloc(rw_uid_cachep, GFP_NOFS);
}

static inline void uid_sub_entry_free(struct rw_uid *uid)
{
	kmem_cache_free(rw_uid_cachep, uid);
}

static inline struct pkg_uid *pkg_entry_alloc(void)
{
	return kmem_cache_alloc(pkg_uid_cachep, GFP_NOFS);
}

static inline void pkg_entry_free(struct pkg_uid *pkg)
{
	kmem_cache_free(pkg_uid_cachep, pkg);
}

static inline void add_to_uid_main(struct rw_uid *entry)
{
	list_add(&entry->rw_uid_list, &g_rw_uid_main_list);
}

static inline void add_to_uid_sub(struct rw_uid *entry)
{
	list_add(&entry->rw_uid_list, &g_rw_uid_sub_list);
}

static inline void add_to_pkglist(struct pkg_uid *entry)
{
	list_add(&entry->pkg_list, &g_pkg_list);
}

/*
 * Remove uid entry from the rw_uid_main list and free the entry
 */
static void remove_rw_main(struct rw_uid *uid)
{
	if (!uid){
		SDFS_ERR("uid is null \n");
		return;
	}
	list_del(&uid->rw_uid_list);
	uid_main_entry_free(uid);
}

/*
 * Remove uid entry from the rw_uid_sub list and free the entry
 */
static void remove_rw_sub(struct rw_uid *uid)
{
	if (!uid){
		SDFS_ERR("uid is null \n");
		return;
	}
	list_del(&uid->rw_uid_list);
	uid_sub_entry_free(uid);
}

/*
 * Remove pkg entry from the pkg list and free the entry
 */
static void remove_pkg(struct pkg_uid *pkg)
{

	if (!pkg){
		SDFS_ERR("pkg is null \n");
		return;
	}
	list_del(&pkg->pkg_list);
	pkg_entry_free(pkg);
}

static void clear_uid_main_list(void)
{
	struct list_head *pos, *n;
	struct rw_uid *uid;
	
	if(list_empty(&g_rw_uid_main_list)){
		return;
	}

	list_for_each_safe(pos, n, &g_rw_uid_main_list){
		uid = list_entry(pos, struct rw_uid,rw_uid_list);
		remove_rw_main(uid);
	}
	
	//TODO: do we need this ?? list_del_init(&cache->cache_list)
}

static void clear_uid_sub_list(void)
{
	struct list_head *pos, *n;
	struct rw_uid *uid;

	if(list_empty(&g_rw_uid_sub_list)){
		return;
	}
	
	list_for_each_safe(pos, n, &g_rw_uid_sub_list){
		uid = list_entry(pos, struct rw_uid,rw_uid_list);
		remove_rw_sub(uid);
	}
	
	//TODO: do we need this ?? list_del_init(&cache->cache_list)
}

static void clear_pkglist(void)
{
	struct list_head *pos, *n;
	struct pkg_uid *pkg;
	
	if(list_empty(&g_pkg_list)){
		return;
	}

	list_for_each_safe(pos, n, &g_pkg_list){
		pkg = list_entry(pos, struct pkg_uid,pkg_list);
		remove_pkg(pkg);
	}
	
	//TODO: do we need this ?? list_del_init(&cache->cache_list)
}


bool get_caller_has_rw_locked(struct sdcardfs_sb_info *sb_info) {
     bool allow = false;
     uid_t uid = current_fsuid(); 

	 appid_t appid = multiuser_get_app_id(uid);
	 
	 if (appid == AID_SHELL) {
	   /* for mtklogger with uid, shell, grant the write permisssion to them */
	   SDFS_DBG("WARNING: appid is AID_SHELL. Grant the write permission to it\n");
	   return true;
	 }
     else if(get_uid_pkg(appid) == NULL) {
		SDFS_DBG("WARNING: appid=%d is NOT in packages.list. Grant the write permission to it\n", appid);
		return true;
     }
     else {
     }	 
     
     allow = check_uid_permission(uid, sb_info->options.write_gid);	
	 SDFS_DBG("uid=%d, appid=%d, allow=%d \n",uid, appid, allow);
     return allow;
   
}

unsigned int check_uid_permission(unsigned int uid, unsigned int write_gid)
{
	struct list_head *pos;
	struct rw_uid 	 *entry;

	if(write_gid == AID_SDCARD_RW){

		//TODO: lock
		if(list_empty(&g_rw_uid_main_list)){
			SDFS_NOTICE("main list empty \n");
			return 0;
		}	
		list_for_each(pos, &g_rw_uid_main_list){
			entry = list_entry(pos, struct rw_uid,rw_uid_list);
			if( (entry->app_uid) == uid){
				//SDFS_DBG("matched@main \n");
				return 1;
			}
		}//list_for_each
	}
	else if(write_gid == AID_MEDIA_RW){
		//TODO: lock
		if(list_empty(&g_rw_uid_sub_list)){
			SDFS_NOTICE("sub list empty \n");
			return 0;
		}
	
		list_for_each(pos, &g_rw_uid_sub_list){
			entry = list_entry(pos, struct rw_uid,rw_uid_list);
			if( (entry->app_uid) == uid){
				//SDFS_DBG("matched@sub \n");
				return 1;
			}
		}//list_for_each
	}
    else {
		SDFS_ERR("Wrong write_gid=%d \n", write_gid);
    }

	return 0;
}

unsigned int get_pkg_uid(const char *name)
{
	struct list_head *pos;
	struct pkg_uid 	 *pkg;

	//TODO: lock
	if(list_empty(&g_pkg_list)){
		SDFS_NOTICE("pkg list empty \n");
		return 0;
	}
	
	list_for_each(pos, &g_pkg_list){
		pkg = list_entry(pos, struct pkg_uid,pkg_list);
		if(strncmp(pkg->pkg_name, name, MAX_NAME_LEN) == 0){
			//SDFS_DBG("matched: '%s' \n", name);
			return(pkg->uid);	
		}
	}
	
	return 0;
}

const char *get_uid_pkg(unsigned int uid)
{
	struct list_head *pos;
	struct pkg_uid 	 *pkg;

	//TODO: lock
	if(list_empty(&g_pkg_list)){
		SDFS_NOTICE("pkg list empty \n");
		return NULL;
	}
	
	list_for_each(pos, &g_pkg_list){
		pkg = list_entry(pos, struct pkg_uid,pkg_list);
		if( pkg->uid == uid){
			//SDFS_DBG("matched, uid=%d name=%s \n",pkg->uid, pkg->pkg_name);
			return(pkg->pkg_name);	
		}
	}
	
	return NULL;
}

static int sdcardfs_clear_proc_read(struct seq_file* m, void* v)
{

	SDFS_NOTICE(" \n");
	

	seq_printf(m, "check_uid_permission: [1000,AID_SDCARD_RW] : %d [1001,AID_SDCARD_RW] : %d\r\n"  ,check_uid_permission(1000,AID_SDCARD_RW)
								, check_uid_permission(1001,AID_SDCARD_RW));
	seq_printf(m, "check_uid_permission: [1000,AID_MEDIA_RW] : %d [1001,AID_MEDIA_RW] : %d\r\n"  ,check_uid_permission(1000,AID_MEDIA_RW)
								, check_uid_permission(1001,AID_MEDIA_RW));
	seq_printf(m, "check_uid_permission: [1000,1011] : %d [1001,1011] : %d\r\n"  ,check_uid_permission(1000,1011)
								, check_uid_permission(1001,1011));
	
	seq_printf(m, "get_pkg_uid: [Hello]: %d [hello] : %d\r\n"  ,get_pkg_uid("Hello"), get_pkg_uid("hello"));
	seq_printf(m, "get_pkg_uid: [World]: %d [world] : %d\r\n"  ,get_pkg_uid("World"), get_pkg_uid("world"));
	seq_printf(m, "get_pkg_uid: [foo]: %d [FOO] : %d\r\n"  ,get_pkg_uid("foo"), get_pkg_uid("FOO"));
	
	return 0;
}

static ssize_t sdcardfs_clear_proc_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
	int 		ret;
	char 		kbuf[1024];
	unsigned int 	clear;
	size_t  	len = 0;
	
	SDFS_NOTICE(" \n");
	
	len = min(count, (sizeof(kbuf)-1));

	if (count == 0)
		return -1;
		
	if(count > sizeof(kbuf))
		count =(sizeof(kbuf)-1);

	ret = copy_from_user(kbuf, buffer, count);
	
	if (ret < 0)
		return -1;
	
	kbuf[count] = '\0';

	sscanf(kbuf, "%d", &clear);

	if(clear){
		SDFS_NOTICE("clear \n");	
		spin_lock(&listlock);	
		clear_pkglist();
		clear_uid_main_list();
		clear_uid_sub_list();
		spin_unlock(&listlock);
	}
	else{
		SDFS_NOTICE("Not clear \n");		
	}
	
	return count;
}

static int sdcardfs_clear_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, sdcardfs_clear_proc_read, NULL);
}

static const struct file_operations sdcardfs_clear_fops = {
	.owner	    = THIS_MODULE,	
	.open       = sdcardfs_clear_proc_open,
	.read       = seq_read, 
	.write      = sdcardfs_clear_proc_write,
};

static int sdcardfs_package2uid_proc_read(struct seq_file* m, void* v)
{
	struct list_head *pos;
	struct pkg_uid *pkg;

	if(list_empty(&g_pkg_list)){
		seq_printf(m, "list empty \n");
		return 0;
	}
	
	list_for_each(pos, &g_pkg_list){
		pkg = list_entry(pos, struct pkg_uid,pkg_list);
		seq_printf(m, "pkg=%s, uid=%d \n",pkg->pkg_name,pkg->uid);
	}
	
	return 0;
}

static int sdcardfs_package2uid_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, sdcardfs_package2uid_proc_read, NULL);
}

static ssize_t sdcardfs_package2uid_proc_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
	int 		ret;
	char 		kbuf[1024];
	char 		pkg_name[MAX_NAME_LEN + 1];
	unsigned int 	uid;
	size_t  	len = 0;
	struct pkg_uid 	*entry;
	
	len = min(count, (sizeof(kbuf)-1));

	if (count == 0)
		return -1;
		
	if(count > sizeof(kbuf))
		count =(sizeof(kbuf)-1);

	ret = copy_from_user(kbuf, buffer, count);
	
	if (ret < 0)
		return -1;
	
	kbuf[count] = '\0';

	
	entry = pkg_entry_alloc();

	sscanf(kbuf, "%s %d",entry->pkg_name,&entry->uid);
	SDFS_DBG("pkg_name=%s, uid=%d \n",entry->pkg_name,entry->uid);

	spin_lock(&listlock);	
	add_to_pkglist(entry);
	spin_unlock(&listlock);	 //TODO: refine the lock, use RCU instead
	
	return count;
}

static const struct file_operations sdcardfs_package2uid_fops = {
	.owner	    = THIS_MODULE,	
	.open       = sdcardfs_package2uid_proc_open,
	.read       = seq_read, 
	.write      = sdcardfs_package2uid_proc_write,
};

static int sdcardfs_uid_main_proc_read(struct seq_file* m, void* v)
{
	struct list_head *pos;
	struct rw_uid *uid;

    if(list_empty(&g_rw_uid_main_list)){
		seq_printf(m, "list empty \n");
		return 0;
	}
		
	seq_printf(m, "uid@main \n");
	list_for_each(pos, &g_rw_uid_main_list){
		uid = list_entry(pos, struct rw_uid,rw_uid_list);
		seq_printf(m, "%d \n",uid->app_uid);
	}
	
	return 0;
}

static int sdcardfs_uid_main_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, sdcardfs_uid_main_proc_read, NULL);
}

static ssize_t sdcardfs_uid_main_proc_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
	int 		ret;
	char 		kbuf[1024];
	unsigned int 	uid;
	size_t  	len = 0;
	struct rw_uid 	*entry;
	
	len = min(count, (sizeof(kbuf)-1));

	if (count == 0)
		return -1;
		
	if(count > sizeof(kbuf))
		count =(sizeof(kbuf)-1);

	ret = copy_from_user(kbuf, buffer, count);
	
	if (ret < 0)
		return -1;
	
	kbuf[count] = '\0';

	
	entry = uid_main_entry_alloc();

	sscanf(kbuf, "%d", &entry->app_uid);
	SDFS_DBG("app_uid=%d \n", entry->app_uid);
	
	spin_lock(&listlock);	
	add_to_uid_main(entry);
	spin_unlock(&listlock);	 //TODO: refine the lock, use RCU instead

	return count;
}

static const struct file_operations sdcardfs_uid_main_fops = {
	.owner	    = THIS_MODULE,	
	.open       = sdcardfs_uid_main_proc_open,
	.read       = seq_read, 
	.write      = sdcardfs_uid_main_proc_write,
};

static int sdcardfs_uid_sub_proc_read(struct seq_file* m, void* v)
{
	struct list_head *pos;
	struct rw_uid *uid;
	
	seq_printf(m, "uid@sub\r\n");	
	if(list_empty(&g_rw_uid_sub_list)){
		seq_printf(m, "list empty \n");
		return 0;
	}
	
	list_for_each(pos, &g_rw_uid_sub_list){
		uid = list_entry(pos, struct rw_uid,rw_uid_list);
		seq_printf(m, "%d \n",uid->app_uid);
	}
	
	return 0;
}

static int sdcardfs_uid_sub_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, sdcardfs_uid_sub_proc_read, NULL);
}

static ssize_t sdcardfs_uid_sub_proc_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
	int 		ret;
	char 		kbuf[1024];
	unsigned int 	uid;
	size_t  	len = 0;
	struct rw_uid 	*entry;
	
	len = min(count, (sizeof(kbuf)-1));
	if (count == 0)
		return -1;
		
	if(count > sizeof(kbuf))
		count =(sizeof(kbuf)-1);

	ret = copy_from_user(kbuf, buffer, count);
	
	if (ret < 0)
		return -1;
	
	kbuf[count] = '\0';

	
	entry = uid_sub_entry_alloc();

	sscanf(kbuf, "%d",&entry->app_uid);
	SDFS_DBG("app_uid=%d \n",entry->app_uid);
	
	spin_lock(&listlock);	
	add_to_uid_sub(entry);
	spin_unlock(&listlock);	 //TODO: refine the lock, use RCU instead

	return count;
}

static const struct file_operations sdcardfs_uid_sub_fops = {
	.owner	    = THIS_MODULE,	
	.open       = sdcardfs_uid_sub_proc_open,
	.read       = seq_read, 
	.write      = sdcardfs_uid_sub_proc_write,
};


static int sdcardfs_debug_proc_read(struct seq_file* m, void* v)
{
	seq_printf(m, "%d\n", g_sdcardfs_debug);	
	return 0;
}

static int sdcardfs_debug_proc_open(struct inode *inode, struct file *file)
{
    return single_open(file, sdcardfs_debug_proc_read, NULL);
}

static ssize_t sdcardfs_debug_proc_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
	int 		ret;
	char 		kbuf[1024];
	size_t  	len = 0;
	bool debug = false;
	
	len = min(count, (sizeof(kbuf)-1));
	if (count == 0)
		return -1;
		
	if(count > sizeof(kbuf))
		count =(sizeof(kbuf)-1);

	ret = copy_from_user(kbuf, buffer, count);
	
	if (ret < 0)
		return -1;
	
	kbuf[count] = '\0';
	sscanf(kbuf, "%d",&debug);	
	if (debug) {
	   g_sdcardfs_debug = true;
	}
	else {
	   g_sdcardfs_debug = false;
	}
	SDFS_DBG("g_sdcardfs_debug=%d \n", g_sdcardfs_debug);

	return count;
}

static const struct file_operations sdcardfs_debug_fops = {
	.owner	    = THIS_MODULE,	
	.open       = sdcardfs_debug_proc_open,
	.read       = seq_read, 
	.write      = sdcardfs_debug_proc_write,
};

static void create_proc_intf(void)
{
	struct proc_dir_entry *sdcardfs_proc_dir = NULL;
	struct proc_dir_entry *procClearEntry;
	struct proc_dir_entry *procPkgEntry;
	struct proc_dir_entry *procUidMainEntry;
	struct proc_dir_entry *procUidSubEntry;
	struct proc_dir_entry *procDebugEntry;

	sdcardfs_proc_dir = proc_mkdir("sdcardfs", NULL);
	
	if (!sdcardfs_proc_dir){
		SDFS_ERR("proc_mkdir fail!");
		return;
	}
	else{
		sdcardfs_proc_dir->uid = AID_MEDIA_RW;
		sdcardfs_proc_dir->gid = AID_MEDIA_RW;

		procClearEntry = proc_create("clear",  S_IRUGO | S_IWUSR | S_IWGRP, sdcardfs_proc_dir, &sdcardfs_clear_fops);
		if(procClearEntry)
		{
			SDFS_NOTICE("successfully create /proc/sdcardfs/clear \n");
		}else{
			SDFS_ERR("failed to create /proc/sdcardfs/clear \n");
			return;
		}

		procClearEntry->uid = AID_MEDIA_RW;
		procClearEntry->gid = AID_MEDIA_RW;		
		procPkgEntry = proc_create("pkguid",  S_IRUGO | S_IWUSR | S_IWGRP, sdcardfs_proc_dir, &sdcardfs_package2uid_fops);
		if(procPkgEntry)
		{
			SDFS_NOTICE("[%s]: successfully create /proc/sdcardfs/pkguid \n");
		}else{
			SDFS_ERR("[%s]: failed to create /proc/sdcardfs/pkguid \n");
			return;
		}

		procPkgEntry->uid = AID_MEDIA_RW;
		procPkgEntry->gid = AID_MEDIA_RW;
		procUidMainEntry = proc_create("rw_main",  S_IRUGO | S_IWUSR | S_IWGRP, sdcardfs_proc_dir, &sdcardfs_uid_main_fops);
		if(procUidMainEntry)
		{
			SDFS_NOTICE("[%s]: successfully create /proc/sdcardfs/rw_main \n");
		}else{
			SDFS_ERR("[%s]: failed to create /proc/sdcardfs/rw_main \n");
			return;
		}

		procUidMainEntry->uid = AID_MEDIA_RW;
		procUidMainEntry->gid = AID_MEDIA_RW;

		procUidSubEntry = proc_create("rw_sub",  S_IRUGO | S_IWUSR | S_IWGRP, sdcardfs_proc_dir, &sdcardfs_uid_sub_fops);
		if(procUidSubEntry)
		{
			SDFS_NOTICE("[%s]: successfully create /proc/sdcardfs/rw_sub \n");
		}else{
			SDFS_ERR("[%s]: failed to create /proc/sdcardfs/rw_sub \n");
			return;
		}

		procUidSubEntry->uid = AID_MEDIA_RW;
		procUidSubEntry->gid = AID_MEDIA_RW;


		g_sdcardfs_debug = false;
		procDebugEntry = proc_create("debug",  S_IRUGO | S_IWUSR | S_IWGRP, sdcardfs_proc_dir, &sdcardfs_debug_fops);
		if(procDebugEntry)
		{
			SDFS_NOTICE("[%s]: successfully create /proc/sdcardfs/debug \n");
		}else{
			SDFS_ERR("[%s]: failed to create /proc/sdcardfs/debug \n");
			return;
		}

		procDebugEntry->uid = AID_MEDIA_RW;
		procDebugEntry->gid = AID_MEDIA_RW;
	}

	pkg_uid_cachep = kmem_cache_create("sdcardfs_pkg_uid",
				sizeof(struct pkg_uid),
				0, 0,
				NULL);
				
	if (pkg_uid_cachep == NULL){
		SDFS_ERR("pkg_uid_cachep NULL \n");
		return;
	}

	rw_uid_cachep = kmem_cache_create("sdcardfs_uid",
				sizeof(struct rw_uid),
				0,0,
				NULL);
				
	if (rw_uid_cachep == NULL){
		SDFS_ERR("rw_uid_cachep NULL \n");
		return;
	}
				
	return;

}


static void destroy_proc_intf(void)
{
	kmem_cache_destroy(pkg_uid_cachep);
	kmem_cache_destroy(rw_uid_cachep);
}


MODULE_AUTHOR("Erez Zadok, Filesystems and Storage Lab, Stony Brook University"
	      " (http://www.fsl.cs.sunysb.edu/)");
MODULE_DESCRIPTION("Wrapfs " SDCARDFS_VERSION
		   " (http://wrapfs.filesystems.org/)");
MODULE_LICENSE("GPL");

module_init(init_sdcardfs_fs);
module_exit(exit_sdcardfs_fs);
