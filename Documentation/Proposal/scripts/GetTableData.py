#!/usr/bin/env python

import string
import logging
import argparse
import os
import sys

from xml.etree import ElementTree

import gdata.spreadsheet.service

def get_client(email, password):
    gd_client = gdata.spreadsheet.service.SpreadsheetsService()
    gd_client.email = email
    gd_client.password = password
    gd_client.source = 'contradict-samplereturnproposaltablecreator-0'
    gd_client.ProgrammaticLogin()
    return gd_client

def get_spreadsheet(gd_client, title):
    feed = gd_client.GetSpreadsheetsFeed()
    for ss in feed.entry:
        if 'Mass and Power' in ss.title.text:
            break
    else:
        logging.error("No spreadsheet found with %s in title", title)
        return None
    return ss

def get_worksheet(gd_client, ss):
    key = ss.id.text.rsplit('/', 1)[1]
    wksht_feed = gd_client.GetWorksheetsFeed(key)
    wksht_id = wksht_feed.entry[0].id.text.rsplit('/', 1)[1]
    rows_feed = gd_client.GetListFeed(key, wksht_id)
    return [dict([(k, v.text) for k,v in x.custom.iteritems()]) for x in rows_feed.entry]

def make_power_table(sheet, table_template, duration):
    # Component & Demand (Watts) & Duty Cycle & Energy Requirement (Watt Hours)
    row_template="        %(itempart)s & %(powerconsumptionestperw)s & %(dutycycle)s & %(energy)5.1f\\\\"
    total = 0.0
    for row in sheet:
        demand = float(row['powerconsumptionestperw'] or '0.0')
        duty = float(row['dutycycle'] or '0.0')
        number = float(row['number'] or '1.0')
        e = duration*duty*demand*number
        row['energy'] = e
        total += e
    rows = "\n".join([row_template%row for row in sheet])
    table = table_template%{'rows':rows, 'total':total, 'duration':duration}
    return table

def make_mass_table(sheet, table_template):
    # Component & Count & Mass (kg)\\
    row_template="        %(itempart)s & %(number)s & %(massperkg)s\\\\"
    total = 0.0
    for row in sheet:
        m = float(row['massperkg'] or '0.0')
        number = float(row['number'] or '1.0')
        total += m*number
    rows = "\n".join([row_template%row for row in sheet])
    table = table_template%{'rows':rows, 'total':total}
    return table

def main(srcdir, outdir, email, password):
    gd_client = get_client(email, password)
    ss = get_spreadsheet(gd_client, "Mass and Power")
    if ss is None:
        return
    sheet = get_worksheet(gd_client, ss)

    with open(os.path.join(srcdir,"massbudget_template.tex")) as mtf:
        mt = mtf.read()
    mtab = make_mass_table(sheet, mt)
    with open(os.path.join(outdir,"massbudget.tex"), "w") as mbf:
        mbf.write(mtab)

    with open(os.path.join(srcdir,"powerbudget_template.tex")) as ptf:
        pt = ptf.read()
    ptab = make_power_table(sheet, pt, 2.0)
    with open(os.path.join(outdir,"powerbudget.tex"), "w") as pbf:
        pbf.write(ptab)

if __name__=="__main__":
    logging.basicConfig(level=logging.INFO)
    with open(sys.argv[1]) as credfile:
        email = credfile.readline()
        password = credfile.readline()
    main(".", ".", email, password)


