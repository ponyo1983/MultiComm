/*
 * ini_doc.c
 *
 *  Created on: Dec 2, 2013
 *      Author: lifeng
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "ini_doc.h"

static void insert_section(struct ini_doc * pDoc, struct ini_section * pSection);
static void insert_key(struct ini_section *pSection, struct ini_key * pKey);
static char* trim_string(char * src, int * length);
static int find_index(char * src, int offset, char c);

struct ini_doc *create_ini_doc(char * file_name) {
	char buffer[256];

	FILE * file = fopen(file_name, "r");

	if (file == NULL)
		return 0;

	struct ini_doc* pIniDoc = malloc(sizeof(struct ini_doc));
	if (pIniDoc == NULL)
		return NULL;

	pIniDoc->sectionNum = 0;
	pIniDoc->sections = NULL;

	struct ini_section * pSection = NULL;

	while (1) {
		char* line = fgets(buffer, sizeof(buffer), file);
		int length = 0;
		int index = -1;
		char * name = NULL;
		char *value = NULL;
		if (line == NULL)
			break;

		line = trim_string(line, &length);
		if (line == NULL)
			continue;
		if (line[0] == ';')
			continue;
		if (line[0] == '[') {
			index = find_index(line, 1, ']');
			if (index > 2) //find a new section
					{
				line[index] = '\0';
				name = (char*) malloc(index);
				strcpy(name, line + 1);
				name[index - 1] = '\0';
				if (pSection != NULL) {
					insert_section(pIniDoc, pSection);
				}
				pSection = (struct ini_section*) malloc(
						sizeof(struct ini_section));
				pSection->next = NULL;
				pSection->name = name;

			}
		} else if (pSection != NULL) {
			index = find_index(line, 0,'=');
			if (index >= 1 && index < length - 1) //find a new key-value
					{
				line[index] = '\0';
				struct ini_key *pKey = (struct ini_key *) malloc(
						sizeof(struct ini_key));
				pKey->next = NULL;
				name = (char*) malloc(index + 1);
				strcpy(name, line);
				name[index] = '\0';
				pKey->key = name;

				value = (char*) malloc(length - index);
				strcpy(value,line+index+1);
				value[length-index-1]='\0';
				pKey->value = value;

				insert_key(pSection,pKey);

			}

		}
	}

	if (pSection != NULL) {
		insert_section(pIniDoc, pSection);
	}
	fclose(file);

	return pIniDoc;
}

void destroy_ini_doc(struct ini_doc * pDoc) {

}

char* get_ini_string(struct ini_doc* doc,char* section,char* key)
{
	 if(doc==NULL) return NULL;
	 int i,j;
	 int secNum=doc->sectionNum;
	 struct ini_section * pSection=doc->sections;
	 char *value=NULL;
	 for(i=0;i<secNum;i++)
	 {
		 if(strcmp(pSection->name,section)==0)
		 {
			 struct ini_key* pKey=pSection->keys;
			 int keyNum=pSection->keyNum;
			 for(j=0;j<keyNum;j++)
			 {
				 if(strcmp(pKey->key,key)==0)
				 {
					 value=pKey->value;
					 break;
				 }
				 pKey=pKey->next;


			 }


			 break;
		 }
		 pSection=pSection->next;
	 }
	return value;
}

int get_ini_int(struct ini_doc* doc,char* section,char* key)
{
	char * string=get_ini_string(doc,section,key);
	if(string==NULL) return -1;
	int value=atoi(string);
	return value;
}

static char* trim_string(char * src, int * length) {
	int len = strlen(src);
	int i;
	char * start = src;
	char * end = &(src[len - 1]);
	for (i = 0; i < len; i++) {
		if (!isblank(src[i]))
			break;
		start++;

	}
	if (start - src >= len) {
		return NULL;
	}
	for (i = len - 1; i >= 0; i--) {
		if (!isblank(src[i]) && (src[i]!='\n') && (src[i]!='\r'))
			break;
		*end = '\0';
		end--;
	}
	*length = end - start + 1;
	return start;

}
static int find_index(char * src, int offset, char c) {
	int index = -1;
	int i;
	int length = strlen(src);
	for (i = offset; i < length; i++) {
		if (src[i] == c) {
			index = i;
			break;
		}
	}
	return index;
}

static void insert_section(struct ini_doc * pDoc, struct ini_section * pSection) {
	if (pDoc->sectionNum == 0) {
		pDoc->sectionNum = 1;
		pDoc->sections = pSection;
	} else {
		pDoc->sectionNum = pDoc->sectionNum + 1;
		pSection->next = pDoc->sections;

		pDoc->sections = pSection;
	}
}

static void insert_key(struct ini_section *pSection, struct ini_key * pKey) {
	if (pSection->keyNum == 0) {
		pSection->keyNum = 1;
		pSection->keys = pKey;
	} else {

		pSection->keyNum = pSection->keyNum + 1;
		pKey->next = pSection->keys;
		pSection->keys = pKey;
	}
}

