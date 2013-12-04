/*
 * ini_doc.h
 *
 *  Created on: Dec 2, 2013
 *      Author: lifeng
 */

#ifndef INI_DOC_H_
#define INI_DOC_H_


struct ini_doc
{
	int sectionNum;
	struct ini_section * sections;

};

struct ini_section
{
	char * name;
	int keyNum;
	struct ini_key* keys;
	struct ini_section * next;
};

struct ini_key
{
	char * key;
	char *value;
	struct ini_key * next;
};


struct ini_doc * create_ini_doc(char * file_name);
void destroy_ini_doc(struct ini_doc * pDoc);


char* get_ini_string(struct ini_doc* doc,char* section,char* key);
int get_ini_int(struct ini_doc* doc,char* section,char * key);

#endif /* INI_DOC_H_ */
